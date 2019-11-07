/**
 * @file
 *
 * \brief  Definition of nodelet for managing the farm simulation
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "farm_nodelet.hpp"
#include "rviz_visualisation.hpp"
#include "farm_common.hpp"
#include "farm_simulator/FarmSimulatorConfig.h"
#include "farm_simulator/Algae.h"
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/ColorRGBA.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <cmath>
#include <csignal>


using namespace std;


PLUGINLIB_EXPORT_CLASS(mfcpp::FarmNodelet, nodelet::Nodelet)

namespace mfcpp {

/*
 * Definition of static varibles
 */
sig_atomic_t volatile FarmNodelet::b_sigint_ = 0;
ros::Timer FarmNodelet::init_timer_ = ros::Timer();
ros::Timer FarmNodelet::main_timer_ = ros::Timer();

/*
 * Definition of member functions
 */
FarmNodelet::FarmNodelet() {}
FarmNodelet::~FarmNodelet() {}

void FarmNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Catch SIGINT (Ctrl+C) stop signal
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = FarmNodelet::sigint_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // Dynamic reconfigure
  reconfigure_initialised_ = false;
  dynamic_reconfigure::Server<farm_simulator::FarmSimulatorConfig>::CallbackType cb;
  cb = boost::bind(&FarmNodelet::reconfigure_cb, this, _1, _2);
  reconf_srv_.setCallback(cb);

  // ROS parameters
  private_nh_.param<float>("main_loop_freq", main_loop_freq_, 1.0);
  private_nh_.param<int>("nbr_lines", nbr_lines_, 1);
  private_nh_.param<float>("offset_lines", offset_lines_, 1.0);
  private_nh_.param<float>("length_lines", length_lines_, 100.0);
  private_nh_.param<float>("thickness_ropes", thickness_ropes_, 0.1);
  private_nh_.param<float>("depth_lines", depth_lines_, 1.0);
  private_nh_.param<float>("depth_water", depth_water_, 5.0);
  private_nh_.param<float>("anchors_diameter", anchors_diameter_, 0.5);
  private_nh_.param<float>("anchors_height", anchors_height_, 0.5);
  private_nh_.param<int>("nbr_buoys", nbr_buoys_, 2);
  private_nh_.param<float>("buoys_diameter", buoys_diameter_, 0.6);

  private_nh_.param<bool>("randomise_lines", randomise_lines_, false);
  private_nh_.param<float>("alga_miss_rate", alga_miss_rate_, 0.0);
  private_nh_.param<float>("phi_lines", phi_lines_, 0.0);
  private_nh_.param<float>("theta_lines", theta_lines_, 0.0);
  private_nh_.param<float>("bnd_phi_lines", bnd_phi_lines_, 0.3);
  private_nh_.param<float>("bnd_theta_lines", bnd_theta_lines_, 0.3);
  private_nh_.param<float>("bnd_gamma_lines", bnd_gamma_lines_, 0.3);

  private_nh_.param<bool>("randomise_algae", randomise_algae_, false);
  private_nh_.param<int>("nbr_algae", nbr_algae_, 1);
  private_nh_.param<float>("width_algae", width_algae_, 0.2);
  private_nh_.param<float>("length_algae", length_algae_, 1.0);
  private_nh_.param<float>("thickness_algae", thickness_algae_, 0.01);
  private_nh_.param<float>("psi_algae", psi_algae_, 0.0);
  private_nh_.param<float>("std_width_algae", std_width_algae_, 0.05);
  private_nh_.param<float>("std_length_algae", std_length_algae_, 0.2);
  private_nh_.param<float>("std_psi_algae", std_psi_algae_, 0.1);

  private_nh_.param<bool>("disp_disease", disp_disease_, false);
  private_nh_.param<float>("disease_ratio", disease_ratio_, 0.5);
  private_nh_.param<int>("height_disease_heatmap", height_disease_heatmap_, 30);
  private_nh_.param<int>("width_disease_heatmap", width_disease_heatmap_, 10);
  private_nh_.param<int>("height_grid_heatmap", height_grid_heatmap_, 6);
  private_nh_.param<int>("width_grid_heatmap", width_grid_heatmap_, 2);

  // Other variables
  init_done_ = false;

  // ROS publishers
  algae_pub_ = private_nh_.advertise<farm_simulator::Algae>("algae", 0);
  rviz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("farm_makers", 0);

  // Create algae lines
  init_timer_ = private_nh_.createTimer(
    ros::Duration(0), &FarmNodelet::init_cb, this, true
  );


  // Main loop
  main_timer_ = private_nh_.createTimer(
    ros::Duration(1/main_loop_freq_), &FarmNodelet::main_cb, this
  );
}


void FarmNodelet::main_cb(const ros::TimerEvent &timer_event)
{
  if (!ros::ok() || ros::isShuttingDown() || b_sigint_)
    return;

  if (init_done_) {
    pub_rviz_markers(1/main_loop_freq_);
    pub_algae();
  }
}


void FarmNodelet::sigint_handler(int s) {
  b_sigint_ = 1;
  init_timer_.stop();
  main_timer_.stop();

  raise(SIGTERM);
}


void FarmNodelet::reconfigure_cb(farm_simulator::FarmSimulatorConfig &config,
  uint32_t level)
{
  if (reconfigure_initialised_) {
    randomise_lines_ = config.randomise_lines;
    phi_lines_ = config.phi_lines;
    theta_lines_ = config.theta_lines;
    bnd_phi_lines_ = config.bnd_phi_lines;
    bnd_theta_lines_ = config.bnd_theta_lines;
    bnd_gamma_lines_ = config.bnd_gamma_lines;
    disp_disease_ = config.disp_disease;

    init_algae_lines();
  } else
    reconfigure_initialised_ = true;
}


void FarmNodelet::init_cb(const ros::TimerEvent &timer_event)
{
  init_algae_lines();
  init_done_ = true;
}


void FarmNodelet::init_algae_lines()
{
  algae_lines_.resize(0);
  algae_lines_.reserve(nbr_lines_);

  for (unsigned int i = 0; i < nbr_lines_; i++) {
    AlgaeLine line;

    init_anchors(line, i);
    line.nbr_buoys = nbr_buoys_;
    line.buoys_diameter = buoys_diameter_;
    init_ropes(line);
    init_algae(line);

    algae_lines_.emplace_back(line);
  }
}


}  // namespace mfcpp
