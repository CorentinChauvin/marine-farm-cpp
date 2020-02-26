/**
 * @file
 *
 * \brief  Definition of output functions
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "camera_nodelet.hpp"
#include "mf_sensors_simulator/CameraOutput.h"
#include "mf_farm_simulator/farm_common.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <random>
#include <vector>

using namespace std;


namespace mfcpp {

void CameraNodelet::publish_rviz_fov()
{
  MarkerArgs args;
  args.stamp = ros::Time::now();
  args.frame_id = camera_frame_;
  args.ns = "FieldOfView";
  args.duration = ros::Duration(1/camera_freq_);
  args.color.r = fov_color_[0];
  args.color.g = fov_color_[1];
  args.color.b = fov_color_[2];
  args.color.a = fov_color_[3];

  visualization_msgs::Marker marker = rviz_marker_triangles(args);
  marker.points.reserve(18);
  marker.colors.resize(6, args.color);

  // Form the limit points of the field of view
  float a = fov_distance_ / sqrt(
    pow(sensor_width_/2, 2) + pow(sensor_height_/2, 2) + pow(focal_length_, 2)
  );
  float m_x[4] = {-1, 1, 1, -1};  // multiplicators to form the points
  float m_y[4] = {-1, -1, 1, 1};
  tf2::Vector3 origin(0, 0, -focal_length_);
  vector<tf2::Vector3> p(4);

  for (int k = 0; k < 4; k++) {
    p[k] = tf2::Vector3(m_x[k] * a * sensor_width_ / 2,
                        m_y[k] * a * sensor_height_ / 2,
                        a * focal_length_);
  }

  // Create the corresponding triangles
  geometry_msgs::Point point;
  marker.points.emplace_back(tf2::toMsg(origin, point));
  marker.points.emplace_back(tf2::toMsg(p[0], point));
  marker.points.emplace_back(tf2::toMsg(p[1], point));

  marker.points.emplace_back(tf2::toMsg(origin, point));
  marker.points.emplace_back(tf2::toMsg(p[1], point));
  marker.points.emplace_back(tf2::toMsg(p[2], point));

  marker.points.emplace_back(tf2::toMsg(origin, point));
  marker.points.emplace_back(tf2::toMsg(p[2], point));
  marker.points.emplace_back(tf2::toMsg(p[3], point));

  marker.points.emplace_back(tf2::toMsg(origin, point));
  marker.points.emplace_back(tf2::toMsg(p[3], point));
  marker.points.emplace_back(tf2::toMsg(p[0], point));

  marker.points.emplace_back(tf2::toMsg(p[0], point));
  marker.points.emplace_back(tf2::toMsg(p[1], point));
  marker.points.emplace_back(tf2::toMsg(p[2], point));
  marker.points.emplace_back(tf2::toMsg(p[0], point));
  marker.points.emplace_back(tf2::toMsg(p[2], point));
  marker.points.emplace_back(tf2::toMsg(p[3], point));

  // Publish the marker
  rviz_pub_.publish(marker);
}


void CameraNodelet::prepare_out_msgs(
  mf_sensors_simulator::CameraOutput &out_msg,
  visualization_msgs::Marker &ray_marker,
  visualization_msgs::Marker &pts_marker)
{
  // Prepare output message
  out_msg.header.stamp = ros::Time::now();
  out_msg.header.frame_id = camera_frame_;

  int nbr_pts = n_pxl_height_*n_pxl_width_;
  out_msg.x.reserve(nbr_pts);
  out_msg.y.reserve(nbr_pts);
  out_msg.z.reserve(nbr_pts);
  out_msg.value.reserve(nbr_pts);

  // Prepare Rviz marker for displaying rays
  ray_marker.header.stamp = ros::Time::now();
  ray_marker.header.frame_id = camera_frame_;
  ray_marker.ns = "Rays";
  ray_marker.lifetime = ros::Duration(1/camera_freq_);
  ray_marker.color.r = 1.0;
  ray_marker.color.g = 0.0;
  ray_marker.color.b = 0.0;
  ray_marker.color.a = 1.0;
  ray_marker.type = visualization_msgs::Marker::LINE_LIST;
  ray_marker.action = visualization_msgs::Marker::ADD;
  ray_marker.scale.x = 0.01;
  ray_marker.points.reserve(nbr_pts);

  // Prepare Rviz marker for displaying hit points
  pts_marker.header.stamp = ros::Time::now();
  pts_marker.header.frame_id = fixed_frame_;
  pts_marker.ns = "Hit point";
  pts_marker.lifetime = ros::Duration(1/camera_freq_);
  pts_marker.color.r = 0.0;
  pts_marker.color.g = 0.0;
  pts_marker.color.b = 1.0;
  pts_marker.color.a = 1.0;
  pts_marker.type = visualization_msgs::Marker::POINTS;
  pts_marker.action = visualization_msgs::Marker::ADD;
  pts_marker.scale.x = 0.02;
  pts_marker.scale.y = 0.02;
  pts_marker.points.reserve(nbr_pts);
  pts_marker.colors.reserve(nbr_pts);
}


void CameraNodelet::add_pt_to_marker(visualization_msgs::Marker &marker,
  const tf2::Vector3 &pt, float color_r, float color_g, float color_b)
{
  geometry_msgs::Point p;
  p.x = pt.getX();
  p.y = pt.getY();
  p.z = pt.getZ();
  marker.points.emplace_back(p);

  std_msgs::ColorRGBA color;
  color.r = color_r;
  color.g = color_g;
  color.b = color_b;
  color.a = 1.0;
  marker.colors.emplace_back(color);
}


void CameraNodelet::add_line_to_marker(visualization_msgs::Marker &marker,
  const tf2::Vector3 &pt1, const tf2::Vector3 &pt2)
{
  geometry_msgs::Point p;
  p.x = pt1.getX();
  p.y = pt1.getY();
  p.z = pt1.getZ();
  marker.points.emplace_back(p);
  p.x = pt2.getX();
  p.y = pt2.getY();
  p.z = pt2.getZ();
  marker.points.emplace_back(p);
}


void CameraNodelet::publish_output()
{
  // Prepare output message
  mf_sensors_simulator::CameraOutput out_msg;
  visualization_msgs::Marker ray_marker;
  visualization_msgs::Marker pts_marker;

  prepare_out_msgs(out_msg, ray_marker, pts_marker);

  // Selects algae that are in field of view of the camera
  coll_mutex_.lock();
  update_fov_pose();
  overlap_fov(fov_body_);

  // Get their position and dimension, and compute their axes
  int n = ray_bodies_.size();
  vector<float> w_algae(n);  // width of algae
  vector<float> h_algae(n);  // height of algae
  vector<float> inc_y3(n);   // increment along y3 axis of algae
  vector<float> inc_z3(n);   // increment along z3 axis of algae
  vector<geometry_msgs::TransformStamped> tf_algae(n);  // transforms of local frames

  get_ray_algae_carac(w_algae, h_algae, inc_y3,  inc_z3, tf_algae);

  int n_heat_height = heatmaps_[0].size();    // height of the heatmap
  int n_heat_width = heatmaps_[0][0].size();  // width of the heatmap

  // Initialise randomisation of heatmap values
  std::random_device seed_initialiser;
  std::mt19937 random_generator(seed_initialiser());

  // For each pixel
  for (unsigned int i = 0; i < n_pxl_height_; i++) {
    for (unsigned int j = 0; j < n_pxl_width_; j++) {
      // Perform ray casting
      tf2::Vector3 aim_pt = get_aim_pt(i, j);

      int alga_idx;
      tf2::Vector3 hit_pt;
      bool alga_hit = raycast_alga(aim_pt, hit_pt, alga_idx, fixed_camera_tf_);

      if (alga_hit) {
        // Transform hit point in alga frame
        geometry_msgs::Pose hit_pose, tf_pose;
        hit_pose.position.x = hit_pt.getX();
        hit_pose.position.y = hit_pt.getY();
        hit_pose.position.z = hit_pt.getZ();
        tf2::doTransform(hit_pose, tf_pose, tf_algae[alga_idx]);
        float z = tf_pose.position.z + h_algae[alga_idx]/2;
        float y = tf_pose.position.y + w_algae[alga_idx]/2;

        // Get alga disease value
        int k = z / inc_z3[alga_idx];
        int l = y / inc_y3[alga_idx];
        k = min(k, n_heat_height-1);  // prevent problems with the last element
        l = min(l, n_heat_width-1);

        float value = heatmaps_[corr_algae_[alga_idx]][k][l];

        // Transform hit point in camera frame
        geometry_msgs::Pose out_pose;
        tf2::doTransform(hit_pose, out_pose, camera_fixed_tf_);

        // Noise disease value
        if (noise_meas_) {
          geometry_msgs::Point p = out_pose.position;
          float distance = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));
          float sigma = noise_std_ * (1 - exp(-distance/noise_decay_));
          std::normal_distribution<float> distribution(0, sigma);

          float noise = distribution(random_generator);
          value += noise;

          if (value > 1.0)
            value = 1.0;
          else if (value < 0.0)
            value = 0.0;
        }

        // Fill output message
        out_msg.x.emplace_back(out_pose.position.x);
        out_msg.y.emplace_back(out_pose.position.y);
        out_msg.z.emplace_back(out_pose.position.z);
        out_msg.value.emplace_back(value);

        // Fill point marker
        add_pt_to_marker(pts_marker, hit_pt, value, value, value);
      }

      // Fill ray marker
      add_line_to_marker(ray_marker, tf2::Vector3(0, 0, 0), aim_pt);
    }
  }

  coll_mutex_.unlock();

  // Publish markers
  out_pub_.publish(out_msg);
  rviz_pub_.publish(pts_marker);
  rviz_pub_.publish(ray_marker);
}


}  // namespace mfcpp
