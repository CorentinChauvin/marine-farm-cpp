/**
 * @file
 *
 * \brief  Definition of a nodelet for a Gaussian Process mapping an
 *         algae wall
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "gp_nodelet.hpp"
#include "sensors_simulator/CameraOutput.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <vector>
#include <iostream>

using namespace std;

PLUGINLIB_EXPORT_CLASS(mfcpp::GPNodelet, nodelet::Nodelet)


namespace mfcpp {

/*
 * Definition of static variables
 */
sig_atomic_t volatile GPNodelet::b_sigint_ = 0;
ros::Timer GPNodelet::main_timer_ = ros::Timer();

/*
 * Definition of member functions
 */
GPNodelet::GPNodelet() {}
GPNodelet::~GPNodelet() {}


void GPNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Catch SIGINT (Ctrl+C) stop signal
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = GPNodelet::sigint_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // ROS parameters
  private_nh_.param<float>("main_freq", main_freq_, 1.0);

  // Other variables
  camera_msg_available_ = false;
  gp_initialised_ = false;

  // ROS subscribers
  // algae_sub_ = private_nh_.subscribe<farm_simulator::Algae>("algae", 1,
  //   boost::bind(&CameraNodelet::algae_cb, this, _1));
  camera_sub_ = nh_.subscribe<sensors_simulator::CameraOutput>("camera_out", 1, &GPNodelet::camera_cb, this);

  // ROS publishers
  // out_pub_ = nh_.advertise<sensors_simulator::CameraOutput>("camera_out", 0);



  // Other parameters


  // Main loop
  main_timer_ = private_nh_.createTimer(
    ros::Duration(1/main_freq_), &GPNodelet::main_cb, this
  );
}


void GPNodelet::main_cb(const ros::TimerEvent &timer_event)
{
  if (!ros::ok() || ros::isShuttingDown() || b_sigint_)
    return;

  if (!gp_initialised_) {
    init_gp();
    gp_initialised_ = true;
  }
  else if (camera_msg_available_) {
    vector<float> x, y, z;
    transform_points(camera_msg_->x, camera_msg_->y, camera_msg_->z, x, y, z);
    update_gp(x, y, z, camera_msg_->value);
  }
}


void GPNodelet::sigint_handler(int s) {
  b_sigint_ = 1;
  main_timer_.stop();
}


void GPNodelet::camera_cb(const sensors_simulator::CameraOutput::ConstPtr &msg)
{
  camera_msg_ = msg;
  camera_msg_available_ = true;
}

void GPNodelet::transform_points(const vec_f &x_in, const vec_f &y_in, const vec_f &z_in,
  vec_f &x_out, vec_f &y_out, vec_f &z_out)
{

}



}  // namespace mfcpp
