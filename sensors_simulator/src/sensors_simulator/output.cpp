/**
 * @file
 *
 * \brief  Definition of output functions
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "camera_nodelet.hpp"
#include "sensors_simulator/CameraOutput.h"
#include "farm_simulator/farm_common.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
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


void CameraNodelet::publish_output()
{
  // Prepare output message
  sensors_simulator::CameraOutput out_msg;
  out_msg.header.stamp = ros::Time::now();
  out_msg.header.frame_id = camera_frame_;

  int nbr_pts = n_pxl_height_*n_pxl_width_;
  out_msg.x.reserve(nbr_pts);
  out_msg.y.reserve(nbr_pts);
  out_msg.z.reserve(nbr_pts);
  out_msg.value.reserve(nbr_pts);

  // Prepare Rviz marker for displaying rays
  visualization_msgs::Marker ray_marker;
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
  visualization_msgs::Marker pts_marker;
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

  // Selects algae that are in field of view of the camera
  overlap_fov();

  // Get their position and dimension, and compute their axes
  int n = ray_bodies_.size();
  vector<float> w_algae(n);  // width of algae
  vector<float> h_algae(n);  // height of algae
  vector<float> inc_y3(n);   // increment along y3 axis of algae
  vector<float> inc_z3(n);   // increment along z3 axis of algae
  vector<geometry_msgs::TransformStamped> tf_algae(n);  // transforms of local frames

  float n_height = heatmaps_[0].size();    // height of the heatmap
  float n_width = heatmaps_[0][0].size();  // width of the heatmap

  for (int k = 0; k < n; k++) {
    // Compute the alga inverse transform
    rp3d::Transform inverse_tf = ray_bodies_[k]->getTransform().getInverse();
    rp3d::Vector3 pos = inverse_tf.getPosition();
    rp3d::Quaternion quati = inverse_tf.getOrientation();
    tf_algae[k].transform.translation.x = pos.x;
    tf_algae[k].transform.translation.y = pos.y;
    tf_algae[k].transform.translation.z = pos.z;
    tf_algae[k].transform.rotation.x = quati.x;
    tf_algae[k].transform.rotation.y = quati.y;
    tf_algae[k].transform.rotation.z = quati.z;
    tf_algae[k].transform.rotation.w = quati.w;

    // Get dimensions and increments
    w_algae[k] = 2*ray_shapes_[k]->getExtent().y;
    h_algae[k] = 2*ray_shapes_[k]->getExtent().z;

    inc_y3[k] = w_algae[k] / n_width;
    inc_z3[k] = h_algae[k] / n_height;
  }

  // For each pixel
  for (unsigned int i = 0; i < n_pxl_height_; i++) {
    for (unsigned int j = 0; j < n_pxl_width_; j++) {
      // Perform ray casting
      tf2::Vector3 a(sensor_width_*(-1./2 + float(j)/(n_pxl_width_-1)),
                     sensor_height_*(-1./2 + float(i)/(n_pxl_height_-1)),
                     focal_length_);
      tf2::Vector3 origin(0, 0, 0);
      tf2::Vector3 p = fov_distance_ / tf2::tf2Distance(a, origin) * a;

      int alga_idx;
      tf2::Vector3 hit_pt;
      bool alga_hit = raycast_alga(p, hit_pt, alga_idx);

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
        float value = heatmaps_[corr_algae_[alga_idx]][k][l];

        // Transform hit point in camera frame
        geometry_msgs::Pose out_pose;
        tf2::doTransform(hit_pose, out_pose, camera_fixed_tf_);

        // Fill output message
        out_msg.x.emplace_back(out_pose.position.x);
        out_msg.y.emplace_back(out_pose.position.y);
        out_msg.z.emplace_back(out_pose.position.z);
        out_msg.value.emplace_back(value);

        // Fill point marker
        geometry_msgs::Point pt;
        pt.x = hit_pt.getX();
        pt.y = hit_pt.getY();
        pt.z = hit_pt.getZ();
        pts_marker.points.emplace_back(pt);

        std_msgs::ColorRGBA color;
        color.r = value;
        color.g = value;
        color.b = value;
        color.a = 1.0;
        pts_marker.colors.emplace_back(color);
      }

      // Fill ray marker
      geometry_msgs::Point pt;
      pt.x = 0.0;
      pt.y = 0.0;
      pt.z = 0.0;
      ray_marker.points.emplace_back(pt);
      pt.x = p.getX();
      pt.y = p.getY();
      pt.z = p.getZ();
      ray_marker.points.emplace_back(pt);
    }
  }

  // Publish markers
  out_pub_.publish(out_msg);
  rviz_pub_.publish(pts_marker);
  rviz_pub_.publish(ray_marker);
}


}  // namespace mfcpp
