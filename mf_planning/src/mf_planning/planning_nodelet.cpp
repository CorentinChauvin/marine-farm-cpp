/**
 * @file
 *
 * \brief  Definition of a nodelet for path plannning of an underwater robot
 *         surveying a marine farm
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "planning_nodelet.hpp"
#include "mf_robot_model/robot_model.hpp"
#include "mf_sensors_simulator/MultiPoses.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <vector>
#include <iostream>


using namespace std;

PLUGINLIB_EXPORT_CLASS(mfcpp::PlanningNodelet, nodelet::Nodelet)


namespace mfcpp {

/*
 * Definition of static variables
 */
sig_atomic_t volatile PlanningNodelet::b_sigint_ = 0;
ros::Timer PlanningNodelet::main_timer_ = ros::Timer();

/*
 * Definition of member functions
 */
PlanningNodelet::PlanningNodelet():
  tf_listener_(tf_buffer_)
{

}


PlanningNodelet::~PlanningNodelet() {}


void PlanningNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Catch SIGINT (Ctrl+C) stop signal
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = PlanningNodelet::sigint_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // ROS parameters
  vector<double> model_csts;  // model constants

  private_nh_.param<float>("main_freq", main_freq_, 1.0);
  private_nh_.param<string>("wall_frame", wall_frame_, "wall");
  private_nh_.param<string>("robot_frame", robot_frame_, "base_link");
  private_nh_.param<int>("nbr_int_steps", nbr_int_steps_, 10);
  private_nh_.param<vector<double>>("model_constants", model_csts, vector<double>(11, 0.0));
  private_nh_.param<float>("max_lat_rudder", max_lat_rudder_, 15.0);
  private_nh_.param<float>("max_elev_rudder", max_elev_rudder_, 10.0);
  private_nh_.param<float>("plan_speed", plan_speed_, 1.0);
  private_nh_.param<float>("plan_horizon", plan_horizon_, 1.0);
  private_nh_.param<float>("lattice_res", lattice_res_, 0.5);
  private_nh_.param<float>("min_wall_dist", min_wall_dist_, 0.3);
  private_nh_.param<int>("camera_width", camera_width_, -1);
  private_nh_.param<int>("camera_height", camera_height_, -1);

  // Other variables
  robot_model_ = RobotModel(model_csts);

  // ROS subscribers

  // ROS publishers
  lattice_pub_ = nh_.advertise<visualization_msgs::Marker>("wp_lattice", 0);
  lattice_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("wp_pose_array", 0);

  // ROS services
  ray_multi_client_ = nh_.serviceClient<mf_sensors_simulator::MultiPoses>("raycast_multi");


  // Main loop
  main_timer_ = private_nh_.createTimer(
    ros::Duration(1/main_freq_), &PlanningNodelet::main_cb, this
  );
}


void PlanningNodelet::main_cb(const ros::TimerEvent &timer_event)
{
  if (!ros::ok() || ros::isShuttingDown() || b_sigint_)
    return;

  if (get_tf()) {
    plan_trajectory();

    // Debugging display
    pub_lattice_markers();
  }
}


void PlanningNodelet::sigint_handler(int s) {
  b_sigint_ = 1;
  main_timer_.stop();

  raise(SIGTERM);
}


void PlanningNodelet::pub_lattice_markers()
{
  // Prepare the Rviz message
  unsigned int n = lattice_.size();

  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = robot_frame_;
  marker.ns = "lattice";
  marker.lifetime = ros::Duration(1/main_freq_);
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.points.resize(n);

  // Fill the message
  for (unsigned int k = 0; k < n; k++) {
    marker.points[k].x = lattice_[k].position.x;
    marker.points[k].y = lattice_[k].position.y;
    marker.points[k].z = lattice_[k].position.z;
  }

  lattice_pub_.publish(marker);

  // Publish the corresponding poses
  geometry_msgs::PoseArray msg;
  msg.header.frame_id = robot_frame_;
  msg.poses = lattice_;
  lattice_pose_pub_.publish(msg);
}


bool PlanningNodelet::get_tf()
{
  geometry_msgs::TransformStamped transform;

  try {
    transform = tf_buffer_.lookupTransform(wall_frame_, robot_frame_, ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    NODELET_WARN("[planning_nodelet] %s", ex.what());
    return false;
  }

  wall_robot_tf_ = transform;
  return true;
}



}  // namespace mfcpp
