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
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <string>
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
GPNodelet::GPNodelet():
  tf_listener_(tf_buffer_)
{

}

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
  private_nh_.param<string>("wall_frame", wall_frame_, "");
  private_nh_.param<float>("camera_var", camera_var_, 1.0);
  private_nh_.param<float>("camera_decay", camera_decay_, 1.0);
  private_nh_.param<float>("matern_length", matern_length_, 1.0);
  private_nh_.param<float>("matern_var", matern_var_, 1.0);
  private_nh_.param<float>("gp_noise_var", gp_noise_var_, 1.0);
  private_nh_.param<float>("size_wall_x", size_wall_x_, 2.0);
  private_nh_.param<float>("size_wall_y", size_wall_y_, 30.0);
  private_nh_.param<int>("size_gp_x", size_gp_x_, 2);
  private_nh_.param<int>("size_gp_y", size_gp_y_, 30);
  private_nh_.param<int>("size_img_x", size_img_x_, 40);
  private_nh_.param<int>("size_img_y", size_img_y_, 600);
  private_nh_.param<int>("batch_size", batch_size_, 1);

  // Other variables
  camera_msg_available_ = false;
  gp_initialised_ = false;
  delta_x_ = size_wall_x_ / size_gp_x_;
  delta_y_ = size_wall_y_ / size_gp_y_;
  size_gp_ = size_gp_x_ * size_gp_y_;

  // ROS subscribers
  // algae_sub_ = private_nh_.subscribe<farm_simulator::Algae>("algae", 1,
  //   boost::bind(&CameraNodelet::algae_cb, this, _1));
  camera_sub_ = nh_.subscribe<sensors_simulator::CameraOutput>("camera_out", 1, &GPNodelet::camera_cb, this);

  // ROS publishers
  // out_pub_ = nh_.advertise<sensors_simulator::CameraOutput>("camera_out", 0);


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

    NODELET_INFO("[gp_nodelet] Gaussian Process initialised.");
  }
  else if (camera_msg_available_) {
    vector<float> x, y, z;
    transform_points(camera_msg_->x, camera_msg_->y, camera_msg_->z,
      x, y, z, camera_msg_->header.frame_id, wall_frame_);

    cout << "Starting update..." << endl;
    clock_t begin = clock();

    update_gp(x, y, camera_msg_->z, camera_msg_->value);

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << " -> update done in: " << elapsed_secs <<  endl;

    camera_msg_available_ = false;
  }
}


void GPNodelet::sigint_handler(int s) {
  b_sigint_ = 1;
  main_timer_.stop();

  raise(SIGTERM);
}


void GPNodelet::camera_cb(const sensors_simulator::CameraOutput::ConstPtr &msg)
{
  camera_msg_ = msg;
  camera_msg_available_ = true;
}


bool GPNodelet::transform_points(const vec_f &x_in, const vec_f &y_in, const vec_f &z_in,
  vec_f &x_out, vec_f &y_out, vec_f &z_out, string frame_in, string frame_out)
{
  // Get the tf transform
  geometry_msgs::TransformStamped tf;

  try {
    tf = tf_buffer_.lookupTransform(frame_out, frame_in, ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    NODELET_WARN("%s",ex.what());
    return false;
  }

  // Transform the points
  unsigned int n = x_in.size();
  x_out.resize(n);
  y_out.resize(n);
  z_out.resize(n);

  for (unsigned int k = 0; k < n; k++) {
    geometry_msgs::Pose p_in, p_out;
    p_in.position.x = x_in[k];
    p_in.position.y = y_in[k];
    p_in.position.z = z_in[k];

    tf2::doTransform(p_in, p_out, tf);

    x_out[k] = p_out.position.x;
    y_out[k] = p_out.position.y;
    z_out[k] = p_out.position.z;
  }

  return true;
}



}  // namespace mfcpp