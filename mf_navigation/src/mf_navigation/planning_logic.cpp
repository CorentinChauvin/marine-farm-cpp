/**
 * @file
 *
 * \brief  Definition of a base class node managing high level planning logic
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#include "planning_logic.hpp"
#include "mf_common/common.hpp"
#include "mf_common/spline.hpp"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>
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


PlanningLogic::PlanningLogic():
  nh_("~")
{
 init_node();
}


PlanningLogic::~PlanningLogic()
{

}


void PlanningLogic::init_node()
{
  // ROS parameters
  string path_frame;    //  frame in which the path is expressed
  string wp_file_name;  // relative path of the file containing the waypoints
  string transition_file_name;  // relative path of the file containing the points delimiting the transitions
  float plan_res;               // spatial resolution (m) of the planned trajectory
  bool initially_disabled;      // whether to disable the planner at the beginning

  nh_.param<float>("main_freq", main_freq_, 1.0);
  nh_.param<string>("path_frame", path_frame, "ocean");
  nh_.param<string>("wp_file_name", wp_file_name, "resources/path.txt");
  nh_.param<string>("transition_file_name", transition_file_name, "resources/transition.txt");
  nh_.param<float>("plan_res", plan_res, 1.0);
  nh_.param<bool>("initially_disabled", initially_disabled, false);

  // Other variables
  transition_ = false;
  odom_received_ = false;
  path_loaded_ = false;
  transition_pts_loaded_ = false;

  path_.header.frame_id = path_frame;
  load_path(wp_file_name, plan_res, path_);
  load_transition_pts(transition_file_name);

  if (initially_disabled) {
    transition_ = true;
    transition_pts_.erase(transition_pts_.begin());
  } else {
    transition_ = false;
  }

  // ROS subscribers
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1, &PlanningLogic::odom_cb, this);

  // ROS publishers
  path_pub_ = nh_.advertise<nav_msgs::Path>("path", 0);

  // ROS services
  disable_planner_client_ = nh_.serviceClient<std_srvs::Empty>("disable_planner");
  enable_planner_client_ = nh_.serviceClient<std_srvs::Empty>("enable_planner");
}


void PlanningLogic::run_node()
{
  if (!path_loaded_ || !transition_pts_loaded_) {
    ROS_ERROR("[planning_logic] Couldn't read transition path, shutting down...");
    return;
  }

  ros::Rate loop_rate(main_freq_);

  while (ros::ok()) {
    ros::spinOnce();

    // Check position of the robot
    if (odom_received_ && transition_pts_.size() > 0) {
      geometry_msgs::Pose pose = find_closest(path_.poses, odom_.pose.pose);
      Eigen::Vector3f pos(pose.position.x, pose.position.y, pose.position.z);
      float distance = (pos - transition_pts_[0]).norm();

      if (distance < 0.5) {
        std_srvs::Empty srv;
        double srv_called;

        if (transition_)
          srv_called = enable_planner_client_.call(srv);
        else
          srv_called = disable_planner_client_.call(srv);

        if (srv_called) {
          transition_pts_.erase(transition_pts_.begin());
          transition_ = !transition_;
        } else
          ROS_WARN("[planning_logic] Enable/Disable call to planner failed");
      }
    }

    // Publish the path if needed
    if (transition_) {
      path_.header.stamp = ros::Time::now();
      path_pub_.publish(path_);
    }

    loop_rate.sleep();
  }
}


void PlanningLogic::load_path(string file_name, float resolution, nav_msgs::Path &path)
{
  // Open path file
  string package_path = ros::package::getPath("mf_navigation");
  string file_path = package_path + '/' + file_name;

  std::ifstream file(file_path.c_str());

  if (!file.is_open()) {
    ROS_WARN("[planning_logic] Path file couldn't be opened at: %s", file_path.c_str());
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


void PlanningLogic::load_transition_pts(std::string file_name)
{
  // Open path file
  string package_path = ros::package::getPath("mf_navigation");
  string file_path = package_path + '/' + file_name;

  std::ifstream file(file_path.c_str());

  if (!file.is_open()) {
    ROS_WARN("[planning_logic] Path file couldn't be opened at: %s", file_path.c_str());
    return;
  }

  // Read the waypoints
  transition_pts_.resize(0);
  int k = 0;

  for (string line; std::getline(file, line); ) {
    if (!line.empty()) {
      transition_pts_.emplace_back(Eigen::Vector3f());
      std::istringstream in(line);
      in >> transition_pts_[k](0) >> transition_pts_[k](1) >> transition_pts_[k](2);

      k++;
    }
  }

  // Cleaning
  file.close();
  transition_pts_loaded_ = true;
}


geometry_msgs::Pose PlanningLogic::find_closest(
  const std::vector<geometry_msgs::PoseStamped> &path,
  const geometry_msgs::Pose &pose)
{
  if (path.size() == 0)
    return geometry_msgs::Pose();

  float min_dist = distance2(path[0].pose.position, pose.position);
  int best_k = 0;

  for (int k = 1; k < path.size(); k++) {
    float d = distance2(path[k].pose.position, pose.position);

    if (d < min_dist) {
      min_dist = d;
      best_k = k;
    }
  }

  return path[best_k].pose;
}


void PlanningLogic::odom_cb(const nav_msgs::Odometry msg)
{
  odom_ = msg;
  odom_received_ = true;
}


}  // namespace mfcpp



/**
 * \brief  Main function called before node initialisation
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning_logic");
  mfcpp::PlanningLogic planning_logic;
  planning_logic.run_node();

  return 0;
}
