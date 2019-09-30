/**
 * @file
 *
 * \brief  Definition of nodelet for managing the farm simulation
 * \author Corentin Chauvin-Hamea
 * \date   2019
 */

#include "farm_nodelet.hpp"
#include "farm_common.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>

using namespace std;


PLUGINLIB_EXPORT_CLASS(mfcpp::FarmNodelet, nodelet::Nodelet)

namespace mfcpp {

FarmNodelet::FarmNodelet() {}
FarmNodelet::~FarmNodelet() {}


void FarmNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // ROS parameters
  private_nh_.param<float>("main_loop_freq", main_loop_freq_, 1.0);
  private_nh_.param<int>("nbr_lines", nbr_lines_, 1);
  private_nh_.param<float>("offset_lines", offset_lines_, 1.0);
  private_nh_.param<float>("length_lines", length_lines_, 100.0);
  private_nh_.param<float>("thickness_lines", thickness_lines_, 0.1);
  private_nh_.param<float>("depth_lines", depth_lines_, 1.0);
  private_nh_.param<float>("depth_water", depth_water_, 5.0);
  private_nh_.param<float>("anchors_diameter", anchors_diameter_, 0.5);
  private_nh_.param<float>("anchors_height", anchors_height_, 0.5);

  // ROS publishers
  rviz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("anchors", 0 );

  // Create algae lines
  init_algae_lines();


  // Main loop
  run_nodelet();
}


void FarmNodelet::run_nodelet()
{
  ros::Rate loop_rate(main_loop_freq_);

  while (ros::ok() && !ros::isShuttingDown()) {
    ros::spinOnce();

    pub_rviz_markers(1/main_loop_freq_);

    loop_rate.sleep();
  }
}


void FarmNodelet::init_algae_lines()
{
  for (unsigned int i = 0; i < nbr_lines_; i++) {
    AlgaeLine line;

    // Initialise anchors
    line.anchor1[0] = 0;
    line.anchor1[1] = i * offset_lines_;
    line.anchor1[2] = -depth_water_;

    line.anchor2[0] = length_lines_;
    line.anchor2[1] = i * offset_lines_;
    line.anchor2[2] = -depth_water_;

    line.anchors_diameter = anchors_diameter_;
    line.anchors_height = anchors_height_;

    // TODO: randomise this
    // Initialise line
    line.line.thickness = thickness_lines_;

    line.line.p1[0] = 0;
    line.line.p1[1] = i * offset_lines_;
    line.line.p1[2] = 0;

    line.line.p2[0] = length_lines_;
    line.line.p2[1] = i * offset_lines_;
    line.line.p2[2] = 0;

    // TODO: populate the algae

    algae_lines_.push_back(line);

  }
}


void FarmNodelet::pub_rviz_markers(float duration) const
{
  // Initialise common marker data
  MarkerArgs args;
  args.stamp = ros::Time::now();
  args.duration = ros::Duration(duration);
  args.ns = "ns";
  args.frame_id = "/world";

  visualization_msgs::MarkerArray markers;

  // Visualise algae lines
  for (unsigned int i = 0; i < algae_lines_.size(); i++) {
    const AlgaeLine *al = &algae_lines_[i];  // for convenience

    // Anchors
    markers.markers.push_back(
      rviz_marker_cylinder(al->anchor1, al->anchors_diameter, al->anchors_height, args)
    );
    markers.markers.push_back(
      rviz_marker_cylinder(al->anchor2, al->anchors_diameter, al->anchors_height, args)
    );

    // Ropes
    markers.markers.push_back(
      rviz_marker_line(al->anchor1, al->line.p1, al->line.thickness, args)
    );
    markers.markers.push_back(
      rviz_marker_line(al->line.p1, al->line.p2, al->line.thickness, args)
    );
    markers.markers.push_back(
      rviz_marker_line(al->anchor2, al->line.p2, al->line.thickness, args)
    );
  }

  // Publish the markers
  pop_marker_ids(markers);
  rviz_pub_.publish(markers);

}


visualization_msgs::Marker FarmNodelet::rviz_marker_line(tf2::Vector3 p1, tf2::Vector3 p2,
  float thickness, const MarkerArgs &common_args) const
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

visualization_msgs::Marker FarmNodelet::rviz_marker_cylinder(tf2::Vector3 p, float diameter,
  float height, const MarkerArgs &common_args) const
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


void FarmNodelet::pop_marker_ids(visualization_msgs::MarkerArray &array) const
{
  unsigned int n = array.markers.size();

  for (unsigned int i = 0; i < n; i++) {
    array.markers[i].id = i;
  }
}

} // namespace mfcpp
