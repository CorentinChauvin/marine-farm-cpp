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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Vector3.h>
#include <ros/ros.h>


namespace mfcpp {

void fill_marker_header(visualization_msgs::Marker &marker,
  const MarkerArgs &common_args)
{
  marker.header.frame_id = common_args.frame_id;
  marker.header.stamp = common_args.stamp;
  marker.ns = common_args.ns;
  marker.lifetime = common_args.duration;
}


visualization_msgs::Marker rviz_marker_line(tf2::Vector3 p1, tf2::Vector3 p2,
  float thickness, const MarkerArgs &common_args)
{
  visualization_msgs::Marker marker;
  fill_marker_header(marker, common_args);
  marker.color = common_args.color;

  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Point p;
  p.x = p1.getX();
  p.y = p1.getY();
  p.z = p1.getZ();
  marker.points.emplace_back(p);
  p.x = p2.getX();
  p.y = p2.getY();
  p.z = p2.getZ();
  marker.points.emplace_back(p);

  marker.scale.x = thickness;

  return marker;
}


visualization_msgs::Marker rviz_marker_line(float thickness,
  const MarkerArgs &common_args)
{
  visualization_msgs::Marker marker;
  fill_marker_header(marker, common_args);
  marker.color = common_args.color;
  marker.scale.x = thickness;

  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;


  return marker;
}


visualization_msgs::Marker rviz_marker_cylinder(tf2::Vector3 p, float diameter,
  float height, const MarkerArgs &common_args)
{
  visualization_msgs::Marker marker;
  fill_marker_header(marker, common_args);
  marker.color = common_args.color;

  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = p.getX();
  marker.pose.position.y = p.getY();
  marker.pose.position.z = p.getZ();

  marker.scale.x = diameter;
  marker.scale.y = diameter;
  marker.scale.z = height;

  return marker;
}


visualization_msgs::Marker rviz_marker_rect(const MarkerArgs &common_args)
{
  visualization_msgs::Marker marker;
  fill_marker_header(marker, common_args);
  marker.color = common_args.color;

  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  marker.color.a = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;

  return marker;
}


visualization_msgs::Marker rviz_marker_rect(tf2::Vector3 p1, tf2::Vector3 p2,
  tf2::Vector3 p3, tf2::Vector3 p4, const MarkerArgs &common_args)
{
  visualization_msgs::Marker marker;
  fill_marker_header(marker, common_args);
  marker.color = common_args.color;

  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  marker.points.reserve(6);
  geometry_msgs::Point point;

  marker.points.emplace_back(tf2::toMsg(p1, point));  // first triangle
  marker.points.emplace_back(tf2::toMsg(p2, point));
  marker.points.emplace_back(tf2::toMsg(p4, point));

  marker.points.emplace_back(tf2::toMsg(p2, point));  // second triangle
  marker.points.emplace_back(tf2::toMsg(p3, point));
  marker.points.emplace_back(tf2::toMsg(p4, point));

  std_msgs::ColorRGBA color;
  color.r = 0.0;
  color.g = 1.0;
  color.b = 0.0;
  color.a = 1.0;
  marker.colors.resize(2, color);
  marker.color.a = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;

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
