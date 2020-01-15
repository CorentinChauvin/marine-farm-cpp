/**
 * @file
 *
 * \brief  Definition of a nodelet for Model Predictive Control of an underwater
 *         robot
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#include "mpc_nodelet.hpp"
#include "mf_robot_model/robot_model.hpp"
#include "mf_common/Float32Array.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>

using namespace std;
using Eigen::MatrixXd;

PLUGINLIB_EXPORT_CLASS(mfcpp::MPCNodelet, nodelet::Nodelet)


namespace mfcpp {

/*
 * Definition of static variables
 */
sig_atomic_t volatile MPCNodelet::b_sigint_ = 0;
ros::Timer MPCNodelet::main_timer_ = ros::Timer();

/*
 * Definition of member functions
 */
MPCNodelet::MPCNodelet():
  tf_listener_(tf_buffer_)
{

}


MPCNodelet::~MPCNodelet() {}


void MPCNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Catch SIGINT (Ctrl+C) stop signal
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = MPCNodelet::sigint_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // ROS parameters
  vector<double> model_csts;  // model constants
  vector<double> bnd_input;   // boundaries on the input

  private_nh_.param<float>("main_freq", main_freq_, 10.0);
  private_nh_.param<float>("nominal_speed", nominal_speed_, 1.0);
  private_nh_.param<float>("time_horizon", time_horizon_, 1.0);
  private_nh_.param<int>("nbr_steps", nbr_steps_, 10);

  private_nh_.param<vector<double>>("model_constants", model_csts, vector<double>(11, 0.0));
  private_nh_.param<vector<double>>("bnd_input", bnd_input, vector<double>(4, 0.0));

  // Other variables
  robot_model_ = RobotModel(model_csts);
  path_received_ = false;
  state_received_ = false;

  // ROS subscribers
  path_sub_ = nh_.subscribe<nav_msgs::PathConstPtr>("path", 1, &MPCNodelet::path_cb, this);
  state_sub_ = nh_.subscribe<mf_common::Float32Array>("state", 1, &MPCNodelet::state_cb, this);

  // ROS publishers
  // pose_pub_ = nh_.advertise<geometry_msgs::Pose>("pose_output", 0);


  // TODO: change that
  // vector<double> x_0 = {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};
  // vector<double> u_0 = {robot_model_.steady_propeller_speed(1), 0, 0, 0};
  // robot_model_.get_lin_matrices(x_0, u_0, Ad_, Bd_);
  // cout << "A: \n" << Ad_ << endl;
  // robot_model_.get_lin_discr_matrices(x_0, u_0, Ad_, Bd_, 0.000001, 100);
  // cout << "Ad: \n" << 1/0.000001*Ad_ << endl;


  // Main loop
  main_timer_ = private_nh_.createTimer(
    ros::Duration(1/main_freq_), &MPCNodelet::main_cb, this
  );

  cout << "Timer passed" << endl;
}


void MPCNodelet::main_cb(const ros::TimerEvent &timer_event)
{
  if (!ros::ok() || ros::isShuttingDown() || b_sigint_)
    return;

  if (path_received_ && state_received_) {
    compute_control(path_, state_, nominal_speed_, time_horizon_, nbr_steps_);
  }
}


void MPCNodelet::sigint_handler(int s) {
  b_sigint_ = 1;
  main_timer_.stop();

  raise(SIGTERM);
}


void MPCNodelet::path_cb(const nav_msgs::PathConstPtr msg)
{
  path_ = *msg;
  path_received_ = true;
}


void MPCNodelet::state_cb(const mf_common::Float32Array msg)
{
  state_ = msg.array;
  state_received_ = true;
}


}  // namespace mfcpp
