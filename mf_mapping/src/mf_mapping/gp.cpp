/**
 * @file
 *
 * \brief  Definition of Gaussian Process functions
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "gp_nodelet.hpp"
#include <iostream>

#include <ctime>

using namespace std;
using Eigen::VectorXf;
using Eigen::MatrixXf;


namespace mfcpp {


void GPNodelet::init_gp()
{
  // Initialise mean and covariance of the GP
  gp_mean_.resize(size_gp_, 1);
  gp_cov_ = MatrixXf::Zero(size_gp_, size_gp_);

  for (unsigned int k = 0; k < size_gp_; k++) {
    gp_mean_(k) = gp_init_mean_;
    gp_cov_(k, k) = 1/gp_noise_var_;
  }

  // Fill the wall coordinates
  double x = 0, y = 0;
  x_coord_.resize(size_gp_);
  y_coord_.resize(size_gp_);

  for (int k = 0; k < size_gp_; k++) {
    x_coord_(k) = x;
    y_coord_(k) = y;
    y += delta_y_;

    if (y >= size_wall_y_) {
      y = 0;
      x += delta_x_;
    }
  }

  // Initialise C matrix and its inverse
  gp_C_.resize(size_gp_, size_gp_);
  double inv_var = 1/gp_noise_var_;

  for (int k = 0; k < size_gp_; k++) {
    for (int l = 0; l <= k; l++) {
      gp_C_(k, l) = matern_kernel(x_coord_(k), y_coord_(k), x_coord_(l), y_coord_(l));
      gp_C_(l, k) = gp_C_(k, l);
    }

    gp_C_(k, k) += inv_var;
  }

  gp_C_inv_ = gp_C_.inverse();
}


void GPNodelet::update_gp(const vec_f &x_meas, const vec_f &y_meas,
  const vec_f &distance, const vec_f &values)
{
  int input_idx = 0;  // index in the input vectors
  int total_size_meas = values.size();

  while (input_idx < total_size_meas) {
    // Prepare a batch of measurement data to process
    int size_meas = min(batch_size_, total_size_meas - input_idx);

    VectorXf z(size_meas);  // measurement vector
    MatrixXf R = MatrixXf::Zero(size_meas, size_meas);  // covariance on the measurents

    for (unsigned int k = 0; k < size_meas; k++) {
      z(k) = values[input_idx + k];
      R(k, k) = camera_noise(distance[input_idx + k]);
    }

    // Compute innovation
    MatrixXf k_meas(size_meas, size_gp_);

    for (unsigned int i = 0; i < size_meas; i++) {
      for (unsigned int j = 0; j < size_gp_; j++) {
        k_meas(i, j) = matern_kernel(
          x_meas[input_idx + i], y_meas[input_idx + i], x_coord_(j), y_coord_(j)
        );
      }
    }

    MatrixXf H = k_meas * gp_C_inv_;
    VectorXf v = z - H * gp_mean_;

    // Compute Kalman gain
    MatrixXf S = H * gp_cov_ * H.transpose() + R;
    MatrixXf K = gp_cov_ * H.transpose() * S.inverse();

    // Update mean and covariance
    gp_mean_ += K * v;
    gp_cov_ = gp_cov_ - K * H * gp_cov_;

    // Update batch index
    input_idx += batch_size_;
  }
}



};
