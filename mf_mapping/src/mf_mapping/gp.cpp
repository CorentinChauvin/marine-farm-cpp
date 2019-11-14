/**
 * @file
 *
 * \brief  Definition of Gaussian Process functions
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "gp_nodelet.hpp"
#include <iostream>
#include <cmath>

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


void GPNodelet::pop_reordered_idx(
  unsigned int size_obs, unsigned int size_nobs,
  float min_x, float max_x, float min_y, float max_y,
  vector<unsigned int> &idx_obs , vector<unsigned int> &idx_nobs)
{
  idx_obs.resize(size_obs);
  idx_nobs.resize(size_nobs);

  // Populate observed states indices
  unsigned int k = 0;

  for (unsigned int i = min_x; i <= max_x; i++) {
    for (unsigned int j = min_y; j <= max_y; j++) {
      idx_obs[k] = i*size_gp_y_ + j;
      k++;
    }
  }

  // Populate not observed states indices
  k = 0;               // will go through idx_obs
  unsigned int l = 0;  // will go through idx_nobs
  unsigned int m = 0;  // will go through the original state

  while (m < size_gp_) {
    if (k < size_obs) {
      if (idx_obs[k] > m) {
        idx_nobs[l] = m;
        l++;
      } else {
        k++;
      }
    }
    else {
      idx_nobs[l] = m;
      l++;
    }

    m++;
  }
}


void GPNodelet::notif_changing_pxls(float min_x, float max_x, float min_y,
  float max_y)
{
  unsigned int k = 0;
  float delta_x = size_wall_x_ / size_img_x_;
  float delta_y = size_wall_y_ / size_img_y_;
  float x = 0, y = 0;

  for (unsigned int i = 0; i < size_img_x_; i++) {
    for (unsigned int j = 0; j < size_img_y_; j++) {
      if (x >= min_x && x <= max_x && y >= min_y && y <= max_y) {
        changed_pxl_[k] = true;
      }

      y += delta_y;
      k++;
    }

    x += delta_x;
    y = 0;
  }
}


void GPNodelet::build_Kalman_objects(
  vector<unsigned int> idx_obs, vector<unsigned int> idx_nobs,
  VectorXf &mu, VectorXf &mu_obs,
  MatrixXf &P, MatrixXf &P_obs, MatrixXf &B,
  MatrixXf &C, MatrixXf &C_inv,
  VectorXf &x_coord, VectorXf &y_coord)
{
  unsigned int size_obs = x_coord.size();
  unsigned int size_nobs = size_gp_ - size_obs;

  for (unsigned int k = 0; k < size_obs; k++) {
    mu_obs(k) = gp_mean_(idx_obs[k]);

    for (unsigned int l = 0; l <= k; l++) {
      P_obs(k, l) = gp_cov_(idx_obs[k], idx_obs[l]);
      P_obs(l, k) = P_obs(k, l);

      C(k, l) = gp_C_(idx_obs[k], idx_obs[l]);
      C(l, k) = C(k, l);
    }

    for (unsigned int l = 0; l < size_nobs; l++) {
      B(k, l) = gp_cov_(idx_obs[k], idx_nobs[l]);
    }

    x_coord(k) = x_coord_(idx_obs[k]);
    y_coord(k) = y_coord_(idx_obs[k]);
  }

  for (unsigned int k = 0; k < size_gp_; k++) {
    unsigned int corr_k;

    if (k < size_obs)
      corr_k = idx_obs[k];
    else
      corr_k = idx_nobs[k - size_obs];

    mu(k) = gp_mean_(corr_k);

    for (unsigned int l = 0; l <= k; l++) {
      unsigned int corr_l;

      if (l < size_obs)
        corr_l = idx_obs[l];
      else
        corr_l = idx_nobs[l - size_obs];

      P(k, l) = gp_cov_(corr_k, corr_l);
      P(l, k) = P(k, l);
    }
  }

  C_inv = C.inverse();
}



void GPNodelet::update_gp(const vec_f &x_meas, const vec_f &y_meas,
  const vec_f &z, const vec_f &distances, const vec_f &values)
{
  // Select the observed part of the state wich is concerned by update
  // Asumption: the state is not affected far from the measurements
  float min_x = *std::min_element(x_meas.begin(), x_meas.end());
  float min_y = *std::min_element(y_meas.begin(), y_meas.end());
  float max_x = *std::max_element(x_meas.begin(), x_meas.end());
  float max_y = *std::max_element(y_meas.begin(), y_meas.end());

  float added_distance = -matern_length_/sqrt(3)
                       * log(matern_thresh_/pow(matern_var_, 2));

  min_x = max(float(0), min_x - added_distance);  // area on the wall affected by the update
  max_x = min(float(size_wall_x_ - delta_x_), max_x + added_distance);
  min_y = max(float(0), min_y - added_distance);
  max_y = min(float(size_wall_y_ - delta_y_), max_y + added_distance);

  unsigned int min_obs_x = min_x / delta_x_;  // indices in the original state
  unsigned int max_obs_x = max_x / delta_x_;
  unsigned int min_obs_y = min_y / delta_y_;
  unsigned int max_obs_y = max_y / delta_y_;
  unsigned int size_obs_x = max_obs_x - min_obs_x + 1;  // sizes of the selected state
  unsigned int size_obs_y = max_obs_y - min_obs_y + 1;
  unsigned int size_obs = size_obs_x * size_obs_y;
  unsigned int size_nobs = size_gp_ - size_obs;

  vector<unsigned int> idx_obs(size_obs, 0);
  vector<unsigned int> idx_nobs(size_nobs, 0);  // complementary to the previous one
  pop_reordered_idx(size_obs, size_nobs, min_obs_x, max_obs_x, min_obs_y, max_obs_y,
    idx_obs, idx_nobs);

  // Notify changing pixels
  notif_changing_pxls(min_x, max_x, min_y, max_y);

  // Build mathematical objects that are needed for Kalman update
  VectorXf mu(size_gp_);      // reordered state
  VectorXf mu_obs(size_obs);  // observed part of the state
  MatrixXf P(size_gp_, size_gp_);      // reordered covariance
  MatrixXf P_obs(size_obs, size_obs);  // observed part of the covariance
  MatrixXf B(size_obs, size_nobs);  // off diagonal block matrix in the covariance
  MatrixXf C(size_obs, size_obs);
  MatrixXf C_inv(size_obs, size_obs);
  VectorXf x_coord(size_obs);
  VectorXf y_coord(size_obs);

  build_Kalman_objects(idx_obs, idx_nobs, mu, mu_obs, P, P_obs, B, C, C_inv,
    x_coord, y_coord);

  // Batch updates
  int input_idx = 0;  // index in the input vectors
  int total_size_meas = values.size();

  while (input_idx < total_size_meas) {
    // Prepare a batch of measurement data to process
    int size_meas = min(batch_size_, total_size_meas - input_idx);

    VectorXf z(size_meas);  // measurement vector
    MatrixXf R = MatrixXf::Zero(size_meas, size_meas);  // covariance on the measurents

    for (unsigned int k = 0; k < size_meas; k++) {
      z(k) = values[input_idx + k];
      R(k, k) = camera_noise(distances[input_idx + k]);
    }

    // Compute innovation
    MatrixXf k_meas(size_meas, size_obs);

    for (unsigned int i = 0; i < size_meas; i++) {
      for (unsigned int j = 0; j < size_obs; j++) {
        k_meas(i, j) = matern_kernel(
          x_meas[input_idx + i], y_meas[input_idx + i], x_coord(j), y_coord(j)
        );
      }
    }

    MatrixXf H_obs = k_meas * C_inv;
    VectorXf v = z - H_obs * mu_obs;

    // Compute Kalman gain
    MatrixXf PHt(size_obs, size_meas);
    MatrixXf BtHt(size_nobs, size_meas);
    MatrixXf L(size_gp_, size_meas);

    PHt = P_obs * H_obs.transpose();
    BtHt = B.transpose() * H_obs.transpose();
    L << PHt,
         BtHt;

    MatrixXf S = H_obs * PHt + R;
    MatrixXf S_inv = S.inverse();
    MatrixXf K = L * S_inv;

    // Update mean and covariance
    mu += K * v;
    P = P - L * S_inv * L.transpose();

    for (unsigned int k = 0; k < size_gp_; k++) {
      unsigned int corr_k;

      if (k < size_obs)
        corr_k = idx_obs[k];
      else
        corr_k = idx_nobs[k - size_obs];

      gp_mean_(corr_k) = mu(k);

      for (unsigned int l = 0; l <= k; l++) {
        unsigned int corr_l;

        if (l < size_obs)
          corr_l = idx_obs[l];
        else
          corr_l = idx_nobs[l - size_obs];

        if (P(k, l) > gp_cov_thresh_) {
          gp_cov_(corr_k, corr_l) = P(k, l);
          gp_cov_(corr_l, corr_k) = P(l, k);
        } else {
          // Threshold low values to optimise multiplication performances
          gp_cov_(corr_k, corr_l) = 0;
          gp_cov_(corr_l, corr_k) = 0;
        }
      }
    }

    // Update batch index
    input_idx += batch_size_;
  }
}



};
