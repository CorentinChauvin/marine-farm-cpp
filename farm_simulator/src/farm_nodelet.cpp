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

  // Dynamic reconfigure
  reconfigure_initialised_ = false;
  dynamic_reconfigure::Server<farm_simulator::FarmSimulatorConfig> reconfigure_server;
  dynamic_reconfigure::Server<farm_simulator::FarmSimulatorConfig>::CallbackType cb;
  cb = boost::bind(&FarmNodelet::reconfigure_callback, this, _1, _2);
  reconfigure_server.setCallback(cb);

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

  private_nh_.param<bool>("randomise_lines", randomise_lines_, false);
  private_nh_.param<float>("mean_phi_lines", mean_phi_lines_, 0.0);
  private_nh_.param<float>("mean_theta_lines", mean_theta_lines_, 0.0);
  private_nh_.param<float>("bnd_phi_lines", bnd_phi_lines_, 0.3);
  private_nh_.param<float>("bnd_theta_lines", bnd_theta_lines_, 0.3);
  private_nh_.param<float>("bnd_gamma_lines", bnd_gamma_lines_, 0.3);

  private_nh_.param<int>("nbr_algae", nbr_algae_, 1);
  private_nh_.param<float>("width_alga", width_alga_, 0.2);
  private_nh_.param<float>("length_alga", length_alga_, 1.0);

  // Other variables

  // ROS publishers
  rviz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("farm", 0 );

  // Create algae lines
  init_algae_lines();


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
  if (reconfigure_initialised_) {
    randomise_lines_ = config.randomise_lines;
    mean_phi_lines_ = config.mean_phi_lines;
    mean_theta_lines_ = config.mean_theta_lines;
    bnd_phi_lines_ = config.bnd_phi_lines;
    bnd_theta_lines_ = config.bnd_theta_lines;
    bnd_gamma_lines_ = config.bnd_gamma_lines;

    init_algae_lines();
  } else
    reconfigure_initialised_ = true;
}

#include <ctime>

void FarmNodelet::init_algae_lines()
{
  algae_lines_.resize(0);
  algae_lines_.reserve(5*nbr_lines_);  // per line: 2 anchors, 2 ropes, 1 line

  double l = depth_water_;
  double L = length_lines_;

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
    double phi = mean_phi_lines_;
    double theta = mean_theta_lines_;

    if (randomise_lines_) {
      phi += random_uniform(-bnd_phi_lines_, bnd_phi_lines_);
      theta += random_uniform(-bnd_theta_lines_, bnd_theta_lines_);
    }

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
    if (randomise_lines_)
      gamma += random_uniform(-bnd_gamma_lines_, bnd_gamma_lines_);

    double ca = cos(alpha);
    double sa = sin(alpha);
    double cgcd = cos(gamma)*cos(delta);
    double expr = sin(beta)*cgcd + cos(beta)*sin(delta);
    double sgcd = sin(gamma)*cos(delta);

    double x2 = -l * ( sa*(expr) - ca*sgcd);
    double y2 = -l * (-ca*(expr) - sa*sgcd);
    double z2 = l * (cos(beta)*cgcd - sin(beta)*sin(delta));

    line.line.p1 = line.anchor1 + tf2::Vector3(x1, y1, z1);
    line.line.p2 = line.anchor2 + tf2::Vector3(x2, y2, z2);

    // Add algae on lines
    line.algae.reserve(nbr_algae_);
    unsigned int n = nbr_algae_;
    tf2::Vector3 X1 = line.line.p1;
    tf2::Vector3 X2 = line.line.p2;

    for (unsigned int k = 1; k <= nbr_algae_; k++) {
      Alga alga;
      alga.position = X1 + float(k)/(n+1) * (X2 - X1);
      alga.orientation = 0.0;
      alga.length = length_alga_;
      alga.width = width_alga_;

      line.algae.emplace_back(alga);
    }

    algae_lines_.emplace_back(line);

  }
}


void FarmNodelet::pub_rviz_markers(float duration) const
{
  // Initialise common marker data
  MarkerArgs args;
  args.stamp = ros::Time::now();
  args.duration = ros::Duration(duration);
  args.frame_id = "/world";

  visualization_msgs::MarkerArray markers;
  unsigned int nbr_assets = nbr_lines_ * (5 + nbr_algae_);
  markers.markers.reserve(nbr_assets);

  // Add anchor markers
  args.ns = "anchors";
  for (unsigned int i = 0; i < nbr_lines_; i++) {
    const AlgaeLine *al = &algae_lines_[i];  // for convenience

    markers.markers.emplace_back(
      rviz_marker_cylinder(al->anchor1, al->anchors_diameter, al->anchors_height, args)
    );
    markers.markers.emplace_back(
      rviz_marker_cylinder(al->anchor2, al->anchors_diameter, al->anchors_height, args)
    );
  }

  // Add ropes
  args.ns = "ropes";
  visualization_msgs::Marker line_marker = rviz_marker_line(thickness_lines_, args);
  line_marker.points.reserve(3 * nbr_lines_);

  for (unsigned int i = 0; i < nbr_lines_; i++) {
    const AlgaeLine *al = &algae_lines_[i];
    geometry_msgs::Point point;

    line_marker.points.emplace_back(tf2::toMsg(al->anchor1, point));
    line_marker.points.emplace_back(tf2::toMsg(al->line.p1, point));
    line_marker.points.emplace_back(tf2::toMsg(al->line.p1, point));
    line_marker.points.emplace_back(tf2::toMsg(al->line.p2, point));
    line_marker.points.emplace_back(tf2::toMsg(al->line.p2, point));
    line_marker.points.emplace_back(tf2::toMsg(al->anchor2, point));
  }

  markers.markers.emplace_back(line_marker);

  // Add algae
  args.ns = "algae";
  visualization_msgs::Marker rect_marker = rviz_marker_rect(args);
  rect_marker.points.reserve(2 * nbr_lines_ * nbr_algae_);
  std_msgs::ColorRGBA color;
  color.r = 0.0;
  color.g = 1.0;
  color.b = 0.0;
  color.a = 1.0;
  rect_marker.colors.resize(2 * nbr_lines_ * nbr_algae_, color);

  for (unsigned int i = 0; i < nbr_lines_; i++) {
    const AlgaeLine *al = &algae_lines_[i];

    tf2::Vector3 X1 = al->line.p1;
    tf2::Vector3 X2 = al->line.p2;
    tf2::Vector3 z(0, 0, 1);
    tf2::Vector3 y = (X2-X1) / tf2::tf2Distance(X1, X2);
    tf2::Vector3 x = tf2::tf2Cross(y, z);
    float H = length_alga_;
    float W = width_alga_;

    for (unsigned int k = 0; k < al->algae.size(); k++) {
      tf2::Vector3 X = al->algae[k].position;
      float psi = al->algae[k].orientation;

      tf2::Vector3 p1 = X - W/2*y;
      tf2::Vector3 p2 = X + W/2*y;
      tf2::Vector3 p3 = p2 - H*(cos(psi)*z - sin(psi)*x);
      tf2::Vector3 p4 = p1 - H*(cos(psi)*z - sin(psi)*x);

      // Add the rectangular alga as two triangles
      geometry_msgs::Point point;
      rect_marker.points.emplace_back(tf2::toMsg(p1, point));  // first triangle
      rect_marker.points.emplace_back(tf2::toMsg(p2, point));
      rect_marker.points.emplace_back(tf2::toMsg(p4, point));

      rect_marker.points.emplace_back(tf2::toMsg(p2, point));  // second triangle
      rect_marker.points.emplace_back(tf2::toMsg(p3, point));
      rect_marker.points.emplace_back(tf2::toMsg(p4, point));
    }
  }

  markers.markers.emplace_back(rect_marker);

  // Publish the markers
  pop_marker_ids(markers);
  rviz_pub_.publish(markers);

}


}  // namespace mfcpp
