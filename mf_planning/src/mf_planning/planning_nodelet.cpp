/**
 * @file
 *
 * \brief  Definition of a nodelet for path plannning of an underwater robot
 *         surveying a marine farm
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "planning_nodelet.hpp"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
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
PlanningNodelet::PlanningNodelet() {}
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
  private_nh_.param<float>("main_freq", main_freq_, 1.0);

  // Other variables

  // ROS subscribers

  // ROS publishers


  // Main loop
  main_timer_ = private_nh_.createTimer(
    ros::Duration(1/main_freq_), &PlanningNodelet::main_cb, this
  );
}


void PlanningNodelet::main_cb(const ros::TimerEvent &timer_event)
{
  if (!ros::ok() || ros::isShuttingDown() || b_sigint_)
    return;

  // Do something
  // ...
  cout << "Alive..." << endl;
}


void PlanningNodelet::sigint_handler(int s) {
  b_sigint_ = 1;
  main_timer_.stop();

  raise(SIGTERM);
}




}  // namespace mfcpp
