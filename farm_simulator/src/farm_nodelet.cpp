/**
 * @file
 *
 * \brief  Definition of nodelet for managing the farm simulation
 * \author Corentin Chauvin-Hamea
 * \date   2019
 */

#include "farm_nodelet.hpp"
#include "rviz_visualisation.hpp"
#include "farm_common.hpp"
#include "farm_simulator/FarmSimulatorConfig.h"
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <cmath>
#include <csignal>


using namespace std;


PLUGINLIB_EXPORT_CLASS(mfcpp::FarmNodelet, nodelet::Nodelet)

namespace mfcpp {

sig_atomic_t volatile FarmNodelet::b_sigint_ = 0;

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

  // ROS parameters
  private_nh_.param<float>("main_loop_freq", main_loop_freq_, 1.0);
  private_nh_.param<int>("nbr_lines", nbr_lines_, 1);
  private_nh_.param<float>("offset_lines", offset_lines_, 1.0);
  private_nh_.param<float>("length_lines", length_lines_, 100.0);
  private_nh_.param<float>("thickness_lines", thickness_lines_, 0.1);
  private_nh_.param<float>("depth_lines", depth_lines_, 1.0);
  private_nh_.param<float>("depth_water", depth_water_, 5.0);
  private_nh_.param<float>("anchors_diameter", anchors_diameter_, 0.5);
  private_nh_.param<float>("anchors_height", anchors_height_, 0.5);

  // Other variables
  reconfigure_initialised_ = false;

  // ROS publishers
  rviz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("farm", 0 );

  // Dynamic reconfigure
  dynamic_reconfigure::Server<farm_simulator::FarmSimulatorConfig> reconfigure_server;
  dynamic_reconfigure::Server<farm_simulator::FarmSimulatorConfig>::CallbackType cb;
  cb = boost::bind(&FarmNodelet::reconfigure_callback, this, _1, _2);
  reconfigure_server.setCallback(cb);

  // Create algae lines
  init_algae_lines(false);


  // Main loop
  run_nodelet();
}


void FarmNodelet::run_nodelet()
{
  ros::Rate loop_rate(main_loop_freq_);

  while (ros::ok() && !ros::isShuttingDown() && !b_sigint_) {
    ros::spinOnce();

    pub_rviz_markers(1/main_loop_freq_);

    loop_rate.sleep();
  }

  exit(1);
}


void FarmNodelet::sigint_handler(int s){
  b_sigint_ = 1;
}


void FarmNodelet::reconfigure_callback(farm_simulator::FarmSimulatorConfig &config,
  uint32_t level)
{
  if (reconfigure_initialised_)
    init_algae_lines(false, config.phi, config.theta);
  else
    reconfigure_initialised_ = true;
}


void FarmNodelet::init_algae_lines(bool randomise, float phi, float theta)
{
  algae_lines_.resize(0);

  float l = depth_water_;
  float L = length_lines_;

  if (randomise) {
    phi = random_uniform(-1.0, 1.0);
    theta = random_uniform(-1.0, 1.0);
  }

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

    // Initialise lines
    line.line.thickness = thickness_lines_;

    double x1 = l * sin(theta) * cos(phi);
    double y1 = l * sin(theta) * sin(phi);
    double z1 = l * cos(theta);

    double    D2 = pow(x1, 2) + pow(L-y1, 2);
    double     d = sqrt(D2 + pow(z1, 2));
    double alpha = atan2(x1, L - y1);
    double  beta = atan2(z1, sqrt(D2));
    double delta = -asin((pow(d, 2) + pow(l, 2) - pow(L, 2)) / (2*d*l));
    double gamma = -acos(1/(cos(beta)*cos(delta)) * (z1/l + sin(beta)*sin(delta)));
    gamma = copysign(gamma, y1);

    double x2 = -l * ( sin(alpha)*(sin(beta)*cos(gamma)*cos(delta) + cos(beta)*sin(delta)) - cos(alpha)*sin(gamma)*cos(delta));
    double y2 = -l * (-cos(alpha)*(sin(beta)*cos(gamma)*cos(delta) + cos(beta)*sin(delta)) - sin(alpha)*sin(gamma)*cos(delta));
    double z2 = l * (cos(beta)*cos(gamma)*cos(delta) - sin(beta)*sin(delta));

    line.line.p1 = line.anchor1 + tf2::Vector3(x1, y1, z1);
    line.line.p2 = line.anchor2 + tf2::Vector3(x2, y2, z2);


    // TODO: populate the algae

    algae_lines_.push_back(line);

  }
}


void FarmNodelet::pub_rviz_markers(float duration) const
{
  // Initialise common marker data
  MarkerArgs args;
  args.stamp = ros::Time::now();
  args.duration = ros::Duration(duration);
  args.ns = "ns";
  args.frame_id = "/world";

  visualization_msgs::MarkerArray markers;

  // Visualise algae lines
  for (unsigned int i = 0; i < algae_lines_.size(); i++) {
    const AlgaeLine *al = &algae_lines_[i];  // for convenience

    // Anchors
    markers.markers.push_back(
      rviz_marker_cylinder(al->anchor1, al->anchors_diameter, al->anchors_height, args)
    );
    markers.markers.push_back(
      rviz_marker_cylinder(al->anchor2, al->anchors_diameter, al->anchors_height, args)
    );

    // Ropes
    markers.markers.push_back(
      rviz_marker_line(al->anchor1, al->line.p1, al->line.thickness, args)
    );
    markers.markers.push_back(
      rviz_marker_line(al->line.p1, al->line.p2, al->line.thickness, args)
    );
    markers.markers.push_back(
      rviz_marker_line(al->anchor2, al->line.p2, al->line.thickness, args)
    );
  }

  // Publish the markers
  pop_marker_ids(markers);
  rviz_pub_.publish(markers);

}


}  // namespace mfcpp
