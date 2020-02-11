/**
 * @file
 *
 * \brief  Definition of Gaussian Process functions
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "gp_nodelet.hpp"
#include "mf_mapping/Array2D.h"
#include "mf_mapping/Float32Array.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>

using namespace std;
using Eigen::VectorXf;
using Eigen::MatrixXf;


namespace mfcpp {

void GPNodelet::publish_gp_state()
{
  // Publish GP mean
  mf_mapping::Float32Array mean_msg;
  mean_msg.data = vector<float>(gp_mean_.data(), gp_mean_.data() + gp_mean_.size());;
  gp_mean_pub_.publish(mean_msg);

  // Publish GP covariance
  mf_mapping::Array2D cov_msg;
  cov_msg.data.resize(size_gp_);

  for (int i = 0; i < size_gp_; i++) {
    cov_msg.data[i].data.resize(size_gp_);

    for (int j = 0; j <= i; j++) {
      cov_msg.data[i].data[j] = gp_cov_(i, j);
      cov_msg.data[j].data[i] = gp_cov_(i, j);
    }
  }

  gp_cov_pub_.publish(cov_msg);
}


void GPNodelet::publish_wall_img()
{
  // Preparing evaluation of the Gaussian Process
  int size_obs = idx_obs_.size();
  VectorXf x_obs;  // x coordinates of the observed state
  VectorXf y_obs;  // y coordinates of the observed state
  VectorXf W;      // vector needed for evaluation

  prepare_eval(idx_obs_, gp_mean_, x_obs, y_obs, W);

  // Evaluate the Gaussian Process on the wall
  float delta_x = size_wall_x_ / size_img_x_;
  float delta_y = size_wall_y_ / size_img_y_;

  unsigned int k = 0;
  float x = 0 , y = 0;

  for (unsigned int i = 0; i < size_img_x_; i++) {
    for (unsigned int j = 0; j < size_img_y_; j++) {
      if (changed_pxl_[k]) {
        out_values_[k] = eval_gp(x, y, x_obs, y_obs, W);
        changed_pxl_[k] = false;
      }

      y += delta_y;
      k++;
    }
    x += delta_x;
    y = 0;
  }

  // Fill output image
  sensor_msgs::Image img;
  img.header.stamp = ros::Time::now();
  img.height = size_img_x_;
  img.width = size_img_y_;
  img.encoding = sensor_msgs::image_encodings::MONO8;
  img.is_bigendian = false;
  img.step = size_img_y_;
  img.data.resize(size_img_);

  for (unsigned int k = 0; k < size_img_; k++) {
    img.data[k] = 255 * out_values_[k];
  }

  wall_img_pub_.publish(img);

  // Fill covariance image
  img.header.stamp = ros::Time::now();
  img.height = size_gp_x_;
  img.width = size_gp_y_;
  img.encoding = sensor_msgs::image_encodings::MONO8;
  img.is_bigendian = false;
  img.step = size_gp_y_;
  img.data.resize(size_gp_);

  for (unsigned int k = 0; k < size_gp_; k++) {
    img.data[k] = (1 - gp_cov_(k, k)*gp_noise_var_) * 255;
  }

  cov_img_pub_.publish(img);
}


}  // namespace mfcpp
