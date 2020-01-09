/**
 * @file
 *
 * \brief  Definition of a nodelet for cheated cartesian control of a robot
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#include "cart_control_nodelet.hpp"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <iostream>

using namespace std;

PLUGINLIB_EXPORT_CLASS(mfcpp::CartControlNodelet, nodelet::Nodelet)


namespace mfcpp {

/*
 * Definition of static variables
 */
sig_atomic_t volatile CartControlNodelet::b_sigint_ = 0;
ros::Timer CartControlNodelet::main_timer_ = ros::Timer();

/*
 * Definition of member functions
 */
CartControlNodelet::CartControlNodelet() {}
CartControlNodelet::~CartControlNodelet() {}


void CartControlNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Catch SIGINT (Ctrl+C) stop signal
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = CartControlNodelet::sigint_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // ROS parameters
  private_nh_.param<float>("main_freq", main_freq_, 10.0);
  private_nh_.param<float>("speed", robot_speed_, 1.0);

  // Other variables
  path_received_  = false;
  path_processed_ = false;
  path_executed_  = true;

  // ROS subscribers
  path_sub_ = nh_.subscribe<nav_msgs::PathConstPtr>("path", 1, &CartControlNodelet::path_cb, this);

  // ROS publishers
  pose_pub_ = nh_.advertise<geometry_msgs::Pose>("pose_output", 0);


  // Main loop
  main_timer_ = private_nh_.createTimer(
    ros::Duration(1/main_freq_), &CartControlNodelet::main_cb, this
  );
}


void CartControlNodelet::main_cb(const ros::TimerEvent &timer_event)
{
  if (!ros::ok() || ros::isShuttingDown() || b_sigint_)
    return;

  if (path_received_) {
    compute_waypoints(path_, robot_speed_, 1/main_freq_, waypoints_);
    idx_waypoints_ = 0;
    path_received_ = false;
  }

  if (idx_waypoints_ < waypoints_.size()) {
    pose_pub_.publish(waypoints_[idx_waypoints_]);
    idx_waypoints_++;
  }
}


void CartControlNodelet::sigint_handler(int s) {
  b_sigint_ = 1;
  main_timer_.stop();

  raise(SIGTERM);
}


void CartControlNodelet::path_cb(const nav_msgs::PathConstPtr msg)
{
  path_ = *msg;
  path_received_ = true;
}


void CartControlNodelet::compute_waypoints(const nav_msgs::Path &path, float speed, float dt,
  vector<geometry_msgs::Pose> &waypoints) const
{
  float resolution = speed * dt;  // desired spatial resolution of the waypoints
  int path_size = path.poses.size();
  float s = 0;  // current curvilinear abscissa
  int idx_path = 0;  // current index in the input path
  int idx_wp = 0;    // current index in the output waypoints
  waypoints.resize(1);
  waypoints[0] = path.poses[0].pose;

  while (idx_path < path_size) {

    // Find the next pose in the path
    int prev_idx_path = idx_path;
    float prev_s = s;

    while (idx_path < path_size && s-prev_s < resolution) {
      idx_path++;

      tf2::Vector3 p1, p2;
      tf2::convert(path.poses[idx_path-1].pose.position, p1);
      tf2::convert(path.poses[idx_path].pose.position, p2);
      s += tf2::tf2Distance(p1, p2);
    }

    // Interpolate if the next pose is too far
    if (idx_path < path_size) {
      if (idx_path - prev_idx_path == 1) {
        // Prepare interpolation
        tf2::Vector3 p1, p2;
        tf2::convert(waypoints[idx_wp].position, p1);
        tf2::convert(path.poses[idx_path].pose.position, p2);

        tf2::Quaternion q1, q2;
        tf2::convert(waypoints[idx_wp].orientation, q1);
        tf2::convert(path.poses[idx_path].pose.orientation, q2);

        float d = tf2::tf2Distance(p1, p2);
        int n = d/resolution;

        // Interpolate
        for (int k = 1; k <= n; k++) {
          geometry_msgs::Pose new_pose;
          tf2::toMsg(   p1 + (p2-p1) * (float(k)/n),              new_pose.position);
          tf2::convert((q1 + (q2-q1) * (float(k)/n)).normalize(), new_pose.orientation);

          waypoints.emplace_back(new_pose);
          idx_wp++;
        }
      } else {
        waypoints.emplace_back(path.poses[idx_path].pose);
      }
    }
  }
}




}  // namespace mfcpp
