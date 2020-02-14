/**
 * @file
 *
 * \brief  Definition of a node to generate a fake trajectory and evaluate the
 *         performance of the control.
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#include "experiment_stats.hpp"
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


ExperimentStatsNode::ExperimentStatsNode():
  nh_("~")
{
 init_node();
}


ExperimentStatsNode::~ExperimentStatsNode()
{

}


void ExperimentStatsNode::init_node()
{
  // ROS parameters
  string out_file_name;  // path of the output file for the results of the test

  nh_.param<float>("main_freq", main_freq_, 1.0);
  nh_.param<string>("out_file_name", out_file_name, "/tmp/control_out.txt");

  // Other variables
  odom_received_ = false;
  path_received_ = false;
  start_time_ = ros::Time::now().toSec();

  out_file_.open(out_file_name);
  if (out_file_.is_open())
    out_file_ << "t, x, y, z, x_ref, y_ref, z_ref, pos_err, orient_err" << endl;
  else
    ROS_WARN("[control_tester] Couldn't open file '%s' in write mode", out_file_name.c_str());

  // ROS subscribers
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1, &ExperimentStatsNode::odom_cb, this);
  path_sub_ = nh_.subscribe<nav_msgs::Path>("path", 1, &ExperimentStatsNode::path_cb, this);

  // ROS publishers
  ref_pub_ = nh_.advertise<geometry_msgs::Pose>("reference", 0);
  error_pub_ = nh_.advertise<mf_common::EulerPose>("error", 0);
}


void ExperimentStatsNode::run_node()
{
  ros::Rate loop_rate(main_freq_);
  ros::Time t_path = ros::Time::now();  // last time path has been published

  while (ros::ok()) {
    ros::spinOnce();
    if (path_received_ && odom_received_) {
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


geometry_msgs::Pose ExperimentStatsNode::find_closest(
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


void ExperimentStatsNode::write_output(
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


void ExperimentStatsNode::odom_cb(const nav_msgs::Odometry msg)
{
  odom_ = msg;
  odom_received_ = true;
}


void ExperimentStatsNode::path_cb(const nav_msgs::Path msg)
{
  path_ = msg;
  path_received_ = true;
}


}  // namespace mfcpp



/**
 * \brief  Main function called before node initialisation
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "experiment_stats");
  mfcpp::ExperimentStatsNode experiment_stats_node;
  experiment_stats_node.run_node();

  return 0;
}
