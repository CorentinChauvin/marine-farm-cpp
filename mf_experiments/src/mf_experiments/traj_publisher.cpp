/**
 * @file
 *
 * \brief  Definition of a node to generate hard-coded trajectories
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#include "traj_publisher.hpp"
#include "mf_common/common.hpp"
#include "mf_common/spline.hpp"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using std::cout;
using std::endl;
using std::string;
using std::vector;


namespace mfcpp {


TrajPublisherNode::TrajPublisherNode():
  nh_("~")
{
 init_node();
}


TrajPublisherNode::~TrajPublisherNode()
{

}


void TrajPublisherNode::init_node()
{
  // ROS parameters
  string path_frame;    //  frame in which the path is expressed
  string wp_file_name;  // relative path of the file containing the waypoints
  float plan_speed;    // planned speed (m/s) of the robot
  float plan_res;      // spatial resolution (m) of the planned trajectory

  nh_.param<float>("path_freq", path_freq_, 1.0);
  nh_.param<string>("path_frame", path_frame, "ocean");
  nh_.param<string>("wp_file_name", wp_file_name, "resources/path.txt");
  nh_.param<float>("plan_res", plan_res, 1.0);

  // Other variables
  path_loaded_ = false;
  path_.header.frame_id = path_frame;
  load_path(wp_file_name, plan_res, path_);

  // ROS publishers
  path_pub_ = nh_.advertise<nav_msgs::Path>("path", 0);
}


void TrajPublisherNode::run_node()
{
  ros::Rate loop_rate(path_freq_);

  while (ros::ok()) {
    ros::spinOnce();

    if (path_loaded_) {
      path_.header.stamp = ros::Time::now();
      path_pub_.publish(path_);
    }

    loop_rate.sleep();
  }
}


void TrajPublisherNode::load_path(string file_name, float resolution, nav_msgs::Path &path)
{
  path_loaded_ = false;

  // Open path file
  string package_path = ros::package::getPath("mf_experiments");
  string file_path = package_path + '/' + file_name;

  std::ifstream file(file_path.c_str());

  if (!file.is_open()) {
    ROS_WARN("[control_tester] Path file couldn't be opened at: %s", file_path.c_str());
    return;
  }

  // Read the waypoints
  vector<Eigen::Vector3f> p(0);  // waypoints positions
  vector<Eigen::Vector3f> o(0);  // waypoints orientations
  int k = 0;

  for (string line; std::getline(file, line); ) {
    if (!line.empty()) {
      p.emplace_back(Eigen::Vector3f());
      o.emplace_back(Eigen::Vector3f());
      std::istringstream in(line);
      in >> p[k](0) >> p[k](1) >> p[k](2) >> o[k](0) >> o[k](1) >> o[k](2);

      k++;
    }
  }

  // Interpolate the path
  Spline spline(p, o, 1.0);

  path_.poses.resize(0);
  bool last_reached = false;
  float t = 0;

  while (!last_reached) {
    Eigen::Vector3f p;
    Eigen::Vector3f o;

    spline.evaluate(t, p, o, last_reached);
    o = o / o.norm();

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = p(0);
    pose.pose.position.y = p(1);
    pose.pose.position.z = p(2);
    to_quaternion(0.0, -asin(o(2)), atan2(o(1), o(0)), pose.pose.orientation);

    path_.poses.emplace_back(pose);
    t += resolution;
  }

  // Cleaning
  file.close();
  path_loaded_ = true;
}


}  // namespace mfcpp



/**
 * \brief  Main function called before node initialisation
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_publisher");
  mfcpp::TrajPublisherNode traj_publisher_node;
  traj_publisher_node.run_node();

  return 0;
}
