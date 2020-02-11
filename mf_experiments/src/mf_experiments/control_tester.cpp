/**
 * @file
 *
 * \brief  Definition of a node to generate a fake trajectory and evaluate the
 *         performance of the control.
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#include "control_tester.hpp"
#include "mf_common/common.hpp"
#include "mf_common/spline.hpp"
#include "mf_common/EulerPose.h"
#include <nav_msgs/Odometry.h>
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


ControlTesterNode::ControlTesterNode():
  nh_("~")
{

}


ControlTesterNode::~ControlTesterNode()
{

}


void ControlTesterNode::init_node()
{
  // ROS parameters
  string path_frame;    //  frame in which the path is expressed
  string wp_file_name;  // relative path of the file containing the waypoints
  string out_file_name;  // path of the output file for the results of the test
  float plan_speed;    // planned speed (m/s) of the robot
  float plan_res;      // spatial resolution (m) of the planned trajectory

  nh_.param<float>("main_freq", main_freq_, 1.0);
  nh_.param<float>("path_freq", path_freq_, 1.0);
  nh_.param<string>("path_frame", path_frame, "ocean");
  nh_.param<string>("wp_file_name", wp_file_name, "resources/path.txt");
  nh_.param<string>("out_file_name", out_file_name, "/tmp/control_out.txt");
  nh_.param<float>("plan_res", plan_res, 1.0);

  // Other variables
  odom_received_ = false;
  path_loaded_ = false;
  path_.header.frame_id = path_frame;
  load_path(wp_file_name, plan_res, path_);
  start_time_ = ros::Time::now().toSec();

  out_file_.open(out_file_name);
  if (out_file_.is_open())
    out_file_ << "t, x, y, z, x_ref, y_ref, z_ref, pos_err, orient_err" << endl;
  else
    ROS_WARN("[control_tester] Couldn't open file '%s' in write mode", out_file_name.c_str());

  // ROS subscribers
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1, &ControlTesterNode::odom_cb, this);

  // ROS publishers
  path_pub_ = nh_.advertise<nav_msgs::Path>("path", 0);
  ref_pub_ = nh_.advertise<geometry_msgs::Pose>("reference", 0);
  error_pub_ = nh_.advertise<mf_common::EulerPose>("error", 0);
}


void ControlTesterNode::run_node()
{
  init_node();
  ros::Rate loop_rate(main_freq_);
  ros::Time t_path = ros::Time::now();  // last time path has been published

  while (ros::ok()) {
    ros::spinOnce();

    if (path_loaded_ && (ros::Time::now() - t_path >= ros::Duration(1/path_freq_))) {
      path_pub_.publish(path_);
      t_path = ros::Time::now();
    }

    if (path_loaded_ && odom_received_) {
      geometry_msgs::Pose reference_pose = find_closest(path_.poses, odom_.pose.pose);
      mf_common::EulerPose error;
      diff_pose(reference_pose, odom_.pose.pose, error);

      write_output(odom_.pose.pose, reference_pose, error);

      ref_pub_.publish(reference_pose);
      error_pub_.publish(error);
    }

    loop_rate.sleep();
  }

  // Close the output CSV file
  if (out_file_.is_open())
    out_file_.close();
}


void ControlTesterNode::load_path(string file_name, float resolution, nav_msgs::Path &path)
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


geometry_msgs::Pose ControlTesterNode::find_closest(
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


void ControlTesterNode::write_output(
  const geometry_msgs::Pose &pose,
  const geometry_msgs::Pose &reference,
  const mf_common::EulerPose &error)
{
  if (!out_file_.is_open())
    return;

  float error_pos = sqrt(error.x*error.x + error.y*error.y + error.z*error.z);
  float error_orient = sqrt(error.pitch*error.pitch + error.yaw*error.yaw);  // the roll is not controlled
  double t = ros::Time::now().toSec() - start_time_;

  out_file_ << t << ", "
            << pose.position.x << ", "
            << pose.position.y << ", "
            << pose.position.z << ", "
            << reference.position.x << ", "
            << reference.position.y << ", "
            << reference.position.z << ", "
            << error_pos << ", "
            << error_orient << endl;
}


void ControlTesterNode::odom_cb(const nav_msgs::Odometry msg)
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
  ros::init(argc, argv, "control_tester");
  mfcpp::ControlTesterNode control_tester_node;
  control_tester_node.run_node();

  return 0;
}
