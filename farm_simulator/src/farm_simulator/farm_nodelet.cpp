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
#include "perlin_noise.hpp"
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
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
  private_nh_.param<float>("width_alga", width_algae_, 0.2);
  private_nh_.param<float>("length_alga", length_algae_, 1.0);
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
  rviz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("farm", 0);

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
  }
}


void FarmNodelet::sigint_handler(int s) {
  b_sigint_ = 1;
  init_timer_.stop();
  main_timer_.stop();
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
  algae_lines_.reserve(5*nbr_lines_);  // per line: 2 anchors, 2 ropes, 1 line

  float l = depth_water_;      // distance seafloor - floating line
  float h = l - depth_lines_;  // distance seafloor - algae line
  float L = length_lines_;

  for (unsigned int i = 0; i < nbr_lines_; i++) {
    AlgaeLine line;

    // Initialise anchors
    line.anchor1[0] = i * offset_lines_;
    line.anchor1[1] = 0;
    line.anchor1[2] = -depth_water_;

    line.anchor2[0] = i * offset_lines_;
    line.anchor2[1] = length_lines_;
    line.anchor2[2] = -depth_water_;

    line.anchors_diameter = anchors_diameter_;
    line.anchors_height = anchors_height_;

    // Initialise buoys
    line.nbr_buoys = nbr_buoys_;
    line.buoys_diameter = buoys_diameter_;

    // Initialise floating ropes
    line.thickness_ropes = thickness_ropes_;
    float phi = phi_lines_;
    float theta = theta_lines_;

    if (randomise_lines_) {
      phi += rand_uniform(-bnd_phi_lines_, bnd_phi_lines_);
      theta += rand_uniform(-bnd_theta_lines_, bnd_theta_lines_);
    }

    float x1 = l * sin(theta) * cos(phi);
    float y1 = l * sin(theta) * sin(phi);
    float z1 = l * cos(theta);

    float    D2 = pow(x1, 2) + pow(L-y1, 2);
    float     d = sqrt(D2 + pow(z1, 2));
    float alpha = atan2(x1, L - y1);
    float  beta = atan2(z1, sqrt(D2));
    float delta = -asin((pow(d, 2) + pow(l, 2) - pow(L, 2)) / (2*d*l));
    float gamma = -acos(1/(cos(beta)*cos(delta)) * (z1/l + sin(beta)*sin(delta)));

    gamma = copysign(gamma, y1);
    if (randomise_lines_)
      gamma += rand_uniform(-bnd_gamma_lines_, bnd_gamma_lines_);

    float ca = cos(alpha);
    float sa = sin(alpha);
    float cgcd = cos(gamma)*cos(delta);
    float expr = sin(beta)*cgcd + cos(beta)*sin(delta);
    float sgcd = sin(gamma)*cos(delta);

    float x2 = -l * ( sa*(expr) - ca*sgcd);
    float y2 = -l * (-ca*(expr) - sa*sgcd);
    float z2 = l * (cos(beta)*cgcd - sin(beta)*sin(delta));

    line.floating_rope.p1 = line.anchor1 + tf2::Vector3(x1, y1, z1);
    line.floating_rope.p2 = line.anchor2 + tf2::Vector3(x2, y2, z2);

    // Initialise algae lines
    x1 = h * sin(theta) * cos(phi);
    y1 = h * sin(theta) * sin(phi);
    z1 = h * cos(theta);
    x2 = -h * ( sa*(expr) - ca*sgcd);
    y2 = -h * (-ca*(expr) - sa*sgcd);
    z2 = h * (cos(beta)*cgcd - sin(beta)*sin(delta));

    line.line.p1 = line.anchor1 + tf2::Vector3(x1, y1, z1);
    line.line.p2 = line.anchor2 + tf2::Vector3(x2, y2, z2);

    // Add algae on lines
    line.algae.reserve(nbr_algae_);
    unsigned int n = nbr_algae_;
    tf2::Vector3 X1 = line.line.p1;
    tf2::Vector3 X2 = line.line.p2;

    perlin_.configure(height_disease_heatmap_, width_disease_heatmap_,
      height_grid_heatmap_, width_grid_heatmap_);
    perlin_.randomise_gradients();


    for (unsigned int k = 1; k <= nbr_algae_; k++) {
      if (!rand_bernoulli(alga_miss_rate_)) {
        Alga alga;

        // Initialise pose and dimensions of alga
        alga.position = X1 + float(k)/(n+1) * (X2 - X1);

        if (randomise_algae_) {
          alga.orientation = rand_gaussian(psi_algae_, std_psi_algae_);
          alga.length = abs(rand_gaussian(length_algae_, std_length_algae_));
          alga.width = abs(rand_gaussian(width_algae_, std_width_algae_));
        } else {
          alga.orientation = psi_algae_;
          alga.length = length_algae_;
          alga.width = width_algae_;
        }

        // Initialise disease heatmap of alga
        alga.disease_heatmap.resize(
          height_disease_heatmap_,
          vector<float>(width_disease_heatmap_, 0)
        );

        perlin_.generate();

        for (int i = 0; i < height_disease_heatmap_; i++) {
          for (int j = 0; j < width_disease_heatmap_; j++) {
            float value = perlin_.evaluate(i, j);
            alga.disease_heatmap[i][j] = disease_ratio_*perlin_.accentuate(value);
          }
        }

        // Add the alga
        line.algae.emplace_back(alga);
      }
    }

    algae_lines_.emplace_back(line);

  }
}


}  // namespace mfcpp
