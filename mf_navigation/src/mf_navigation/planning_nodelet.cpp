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
#include "mf_mapping/UpdateGP.h"
#include "mf_common/Float32Array.h"
#include "mf_common/Array2D.h"
#include "mf_common/Float32Array.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
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
  vector<double> model_csts; // model constants
  vector<double> bnd_input;  // boundaries on the input

  private_nh_.param<float>("main_freq", main_freq_, 1.0);
  private_nh_.param<string>("ocean_frame", ocean_frame_, "ocean");
  private_nh_.param<string>("wall_frame", wall_frame_, "wall");
  private_nh_.param<string>("robot_frame", robot_frame_, "base_link");
  private_nh_.param<string>("camera_frame", camera_frame_, "camera");

  private_nh_.param<vector<double>>("model_constants", model_csts, vector<double>(11, 0.0));
  private_nh_.param<vector<double>>("bnd_input", bnd_input, vector<double>(4, 0.0));

  private_nh_.param<bool>("horiz_motion", horiz_motion_, true);
  private_nh_.param<bool>("vert_motion", vert_motion_, true);
  private_nh_.param<bool>("mult_lattices", mult_lattices_, true);
  private_nh_.param<int>("nbr_lattices", nbr_lattices_, 1);
  private_nh_.param<bool>("cart_lattice", cart_lattice_, false);
  private_nh_.param<float>("plan_speed", plan_speed_, 1.0);
  private_nh_.param<float>("plan_horizon", plan_horizon_, 1.0);
  private_nh_.param<int>("lattice_size_horiz", lattice_size_horiz_, 1);
  private_nh_.param<int>("lattice_size_vert", lattice_size_vert_, 1);
  private_nh_.param<float>("wall_orientation", wall_orientation_, 0);
  private_nh_.param<float>("lattice_res", lattice_res_, 0.5);
  private_nh_.param<vector<float>>("bnd_wall_dist", bnd_wall_dist_, {0.5, 1.0});
  private_nh_.param<vector<float>>("bnd_depth", bnd_depth_, {0.0, 1.0});
  private_nh_.param<float>("bnd_pitch", bnd_pitch_, 90.0);

  private_nh_.param<int>("camera_width", camera_width_, -1);
  private_nh_.param<int>("camera_height", camera_height_, -1);
  private_nh_.param<float>("gp_weight", gp_weight_, 1.0);

  private_nh_.param<bool>("linear_path", linear_path_, false);
  private_nh_.param<float>("path_res", path_res_, 0.1);

  wall_orientation_ = fabs(wall_orientation_);
  max_lat_rudder_ = bnd_input[1];
  max_elev_rudder_ = bnd_input[2];
  bnd_pitch_ *= M_PI / 180.0;  // convert in radian

  if (!vert_motion_)
    lattice_size_vert_ = 0;
  if (!horiz_motion_)
    lattice_size_horiz_ = 0;
  if (!mult_lattices_)
    nbr_lattices_ = 1;

  // Other variables
  robot_model_ = RobotModel(model_csts);
  last_gp_mean_.resize(0);
  last_gp_cov_.resize(0);
  x_hit_pt_sel_.resize(0);
  y_hit_pt_sel_.resize(0);
  z_hit_pt_sel_.resize(0);
  state_received_ = false;
  planner_enabled_ = true;


  // ROS subscribers
  gp_mean_sub_ = nh_.subscribe<mf_common::Float32ArrayConstPtr>("gp_mean", 1, &PlanningNodelet::gp_mean_cb, this);
  gp_cov_sub_ = nh_.subscribe<mf_common::Array2DConstPtr>("gp_cov", 1, &PlanningNodelet::gp_cov_cb, this);
  state_sub_ = nh_.subscribe<mf_common::Float32Array>("state", 1, &PlanningNodelet::state_cb, this);

  // ROS publishers
  lattice_pub_ = nh_.advertise<visualization_msgs::Marker>("wp_lattice", 0);
  lattice_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("wp_pose_array", 0);
  path_pub_ = nh_.advertise<nav_msgs::Path>("path", 0);

  // ROS services
  ray_multi_client_ = nh_.serviceClient<mf_sensors_simulator::MultiPoses>("raycast_multi");
  update_gp_client_ = nh_.serviceClient<mf_mapping::UpdateGP>("update_gp");
  disable_serv_ = nh_.advertiseService("disable_planner", &PlanningNodelet::disable_cb, this);
  enable_serv_ = nh_.advertiseService("enable_planner", &PlanningNodelet::enable_cb, this);


  // Main loop
  main_timer_ = private_nh_.createTimer(
    ros::Duration(1/main_freq_), &PlanningNodelet::main_cb, this
  );
}


void PlanningNodelet::main_cb(const ros::TimerEvent &timer_event)
{
  if (!ros::ok() || ros::isShuttingDown() || b_sigint_)
    return;

  if (planner_enabled_ && state_received_ && get_tf()) {
    clock_t start = clock();
    cout << "Starting plannning..." << endl;

    bool success = plan_trajectory();

    cout << "Ellapsed time: " << (clock() - start) / (double)CLOCKS_PER_SEC << endl;

    if (success) {
      path_pub_.publish(path_);
      pub_lattice_markers();
    }
  }
}


void PlanningNodelet::sigint_handler(int s)
{
  b_sigint_ = 1;
  main_timer_.stop();

  raise(SIGTERM);
}


void PlanningNodelet::pub_lattice_markers()
{
  // Prepare lattice Rviz message
  unsigned int n = lattice_.size();

  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = ocean_frame_;
  marker.ns = "lattice";
  marker.lifetime = ros::Duration(1/main_freq_);
  marker.pose.orientation.w = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.points.resize(n);
  marker.colors.resize(n);

  // Publish the non-selected viewpoints
  for (unsigned int k = 0; k < n; k++) {
    marker.points[k].x = lattice_[k].position.x;
    marker.points[k].y = lattice_[k].position.y;
    marker.points[k].z = lattice_[k].position.z;

    marker.colors[k].r = 0.0;
    marker.colors[k].g = 0.0;
    marker.colors[k].b = 1.0;
    marker.colors[k].a = 1.0;
  }

  lattice_pub_.publish(marker);

  // Publish the corresponding poses
  geometry_msgs::PoseArray msg;
  msg.header.frame_id = ocean_frame_;
  msg.poses = lattice_;
  lattice_pose_pub_.publish(msg);

  // Publish the selected viewpoints
  marker.ns = "selected_vp";
  marker.points.resize(nbr_lattices_);
  marker.colors.resize(nbr_lattices_);

  for (int k = 0; k < nbr_lattices_; k++) {
    marker.points[k].x = selected_vp_[k].position.x;
    marker.points[k].y = selected_vp_[k].position.y;
    marker.points[k].z = selected_vp_[k].position.z;

    marker.colors[k].r = 0.0;
    marker.colors[k].g = 1.0;
    marker.colors[k].b = 0.0;
    marker.colors[k].a = 1.0;
  }

  lattice_pub_.publish(marker);

  // Publish the hitpoints of the selected viewpoint
  marker.ns = "hitpoints";
  marker.header.frame_id = ocean_frame_;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  int n_pts = x_hit_pt_sel_.size();
  marker.points.resize(n_pts);
  for (unsigned int k = 0; k < n_pts; k++) {
    marker.points[k].x = x_hit_pt_sel_[k];
    marker.points[k].y = y_hit_pt_sel_[k];
    marker.points[k].z = z_hit_pt_sel_[k];
  }

  lattice_pub_.publish(marker);
}


bool PlanningNodelet::get_tf()
{
  geometry_msgs::TransformStamped t1, t2, t3, t4, t5, t6;

  try {
    t1 = tf_buffer_.lookupTransform(wall_frame_,  robot_frame_, ros::Time(0));
    t2 = tf_buffer_.lookupTransform(ocean_frame_, robot_frame_, ros::Time(0));
    t3 = tf_buffer_.lookupTransform(robot_frame_, ocean_frame_, ros::Time(0));
    t4 = tf_buffer_.lookupTransform(ocean_frame_, wall_frame_,  ros::Time(0));
    t5 = tf_buffer_.lookupTransform(wall_frame_, ocean_frame_,  ros::Time(0));
    t6 = tf_buffer_.lookupTransform(ocean_frame_, camera_frame_,  ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    NODELET_WARN("[planning_nodelet] %s", ex.what());
    return false;
  }

  wall_robot_tf_   = t1;
  ocean_robot_tf_  = t2;
  robot_ocean_tf_  = t3;
  ocean_wall_tf_   = t4;
  wall_ocean_tf_   = t5;
  ocean_camera_tf_ = t6;
  return true;
}


void PlanningNodelet::gp_mean_cb(const mf_common::Float32ArrayConstPtr msg)
{
  last_gp_mean_ = msg->data;
}


void PlanningNodelet::gp_cov_cb(const mf_common::Array2DConstPtr msg)
{
  last_gp_cov_ = array_to_vector2D(msg->data);
}


void PlanningNodelet::state_cb(const mf_common::Float32Array msg)
{
  state_ = RobotModel::state_type(msg.data.begin(), msg.data.end());
  state_received_ = true;
}


bool PlanningNodelet::disable_cb(
  std_srvs::Empty::Request &req,
  std_srvs::Empty::Response &res)
{
  ROS_INFO("[planning_nodelet] Planner disabled");
  planner_enabled_ = false;
  return true;
}


bool PlanningNodelet::enable_cb(
  std_srvs::Empty::Request &req,
  std_srvs::Empty::Response &res)
{
  ROS_INFO("[planning_nodelet] Planner enabled");
  planner_enabled_ = true;
  return true;
}



}  // namespace mfcpp
