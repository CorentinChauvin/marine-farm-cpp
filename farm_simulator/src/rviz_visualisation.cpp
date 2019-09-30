/**
 * @file
 *
 * \brief  Definition of common Rviz display functions
 * \author Corentin Chauvin-Hamea
 * \date   2019
 */

#include "rviz_visualisation.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Vector3.h>
#include <ros/ros.h>


namespace mfcpp {

visualization_msgs::Marker rviz_marker_line(tf2::Vector3 p1, tf2::Vector3 p2,
  float thickness, const MarkerArgs &common_args)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = common_args.frame_id;
  marker.header.stamp = common_args.stamp;
  marker.ns = common_args.ns;
  marker.lifetime = common_args.duration;

  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Point p;
  p.x = p1.getX();
  p.y = p1.getY();
  p.z = p1.getZ();
  marker.points.push_back(p);
  p.x = p2.getX();
  p.y = p2.getY();
  p.z = p2.getZ();
  marker.points.push_back(p);

  marker.scale.x = thickness;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::Marker rviz_marker_cylinder(tf2::Vector3 p, float diameter,
  float height, const MarkerArgs &common_args)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = common_args.frame_id;
  marker.header.stamp = common_args.stamp;
  marker.ns = common_args.ns;
  marker.lifetime = common_args.duration;

  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = p.getX();
  marker.pose.position.y = p.getY();
  marker.pose.position.z = p.getZ();

  marker.scale.x = diameter;
  marker.scale.y = diameter;
  marker.scale.z = height;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  return marker;
}


void pop_marker_ids(visualization_msgs::MarkerArray &array)
{
  unsigned int n = array.markers.size();

  for (unsigned int i = 0; i < n; i++) {
    array.markers[i].id = i;
  }
}


}  // namespace mfcpp
