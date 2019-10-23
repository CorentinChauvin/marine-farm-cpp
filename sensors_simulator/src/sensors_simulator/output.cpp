/**
 * @file
 *
 * \brief  Definition of output functions
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "camera_nodelet.hpp"
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
  ray_marker.points.reserve(n_pxl_height_*n_pxl_width_);

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
  pts_marker.points.reserve(n_pxl_height_*n_pxl_width_);
  pts_marker.colors.reserve(n_pxl_height_*n_pxl_width_);

  // Selects algae that are in field of view of the camera
  overlap_fov();

  // Get their position and dimension, and compute their axes
  int n = ray_bodies_.size();
  vector<tf2::Vector3> origin_algae(n);  // top left corner of algae
  vector<float> inc_y3(n);  // increment along y3 axis of algae
  vector<float> inc_z3(n);  // increment along z3 axis of algae

  float n_height = heatmaps_[0].size();    // height of the heatmap
  float n_width = heatmaps_[0][0].size();  // width of the heatmap

  for (int k = 0; k < n; k++) {
    rp3d::Vector3 pos = ray_bodies_[k]->getTransform().getPosition();
    rp3d::Quaternion quati = ray_bodies_[k]->getTransform().getOrientation();
    tf2::Matrix3x3 rotation(tf2::Quaternion(quati.x, quati.y, quati.z, quati.w));

    tf2::Vector3 y3 = rotation.getColumn(1);
    tf2::Vector3 z3 = rotation.getColumn(2);
    float w_alga = 2*ray_shapes_[k]->getExtent().y;
    float h_alga = 2*ray_shapes_[k]->getExtent().z;

    origin_algae[k] = tf2::Vector3(pos.x, pos.y, pos.z) - w_alga/2*y3 - h_alga/2*z3;

    inc_y3[k] = w_alga / n_width;
    inc_z3[k] = h_alga / n_height * tf2::tf2Dot(z3, tf2::Vector3(0, 0, 1));
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
        // Get alga disease value
        int k = (hit_pt.getZ() - origin_algae[alga_idx].getZ()) / inc_z3[alga_idx];
        int l = (hit_pt.getY() - origin_algae[alga_idx].getY()) / inc_y3[alga_idx];
        float value = heatmaps_[corr_algae_[alga_idx]][k][l];

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
  rviz_pub_.publish(pts_marker);
  rviz_pub_.publish(ray_marker);
}


}  // namespace mfcpp
