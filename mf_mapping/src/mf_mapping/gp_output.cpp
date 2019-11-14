/**
 * @file
 *
 * \brief  Definition of Gaussian Process functions
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "gp_nodelet.hpp"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>

using namespace std;


namespace mfcpp {

void GPNodelet::publish_wall_img()
{
  // Evaluate the Gaussian Process on the wall
  float delta_x = size_wall_x_ / size_img_x_;
  float delta_y = size_wall_y_ / size_img_y_;
  Eigen::VectorXf vect = gp_C_inv_ * gp_mean_;

  unsigned int k = 0;
  float x = 0 , y = 0;

  for (unsigned int i = 0; i < size_img_x_; i++) {
    for (unsigned int j = 0; j < size_img_y_; j++) {
      if (changed_pxl_[k]) {
        Eigen::VectorXf k_pxl(size_gp_);

        for (unsigned int l = 0; l < size_gp_; l++) {
          k_pxl(l) = matern_kernel(
            x, y, x_coord_(l), y_coord_(l)
          );
        }

        out_values_[k] = k_pxl.dot(vect);
        if (out_values_[k] > 1.0)
          out_values_[k] = 1.0;
        else if (out_values_[k] < 0.0)
          out_values_[k] = 0.0;

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
    img.data[k] = out_values_[k] * 255;
  }

  wall_img_pub_.publish(img);
}


}  // namespace mfcpp
