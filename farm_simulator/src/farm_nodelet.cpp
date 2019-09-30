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

FarmNodelet::FarmNodelet() {}
FarmNodelet::~FarmNodelet() {}

sig_atomic_t volatile FarmNodelet::b_sigint_ = 0;


void FarmNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Catch Ctrl+C stop signal
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
    cout << ros::ok() << " ; " << ros::isShuttingDown() << endl;

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

  cout << "*** " << phi << " ; " << theta << endl;
  if (randomise) {
    phi = random_uniform(-1.0, 1.0);
    theta = random_uniform(-1.0, 1.0);
    // gamma = random_uniform(-0.5, 0.5);
  }

  cout << "*** " << phi << " ; " << theta << endl;

  float gamma;

  for (unsigned int i = 0; i < nbr_lines_; i++) {
    AlgaeLine line;

    // Initialise anchors
    line.anchor1[0] = 0;
    line.anchor1[1] = i * offset_lines_;
    line.anchor1[2] = -depth_water_;

    line.anchor2[0] = length_lines_;
    line.anchor2[1] = i * offset_lines_;
    line.anchor2[2] = -depth_water_;

    line.anchors_diameter = anchors_diameter_;
    line.anchors_height = anchors_height_;

    // Initialise lines
    line.line.thickness = thickness_lines_;

    float x1 = l * sin(theta) * cos(phi);
    float y1 = l * sin(theta) * sin(phi);
    float z1 = l * cos(theta);
    float alpha = atan2(x1, L - y1);
    float beta = atan2(z1, sqrt(pow(x1, 2) + pow(L-y1, 2)));
    cout << "beta=" << beta << endl;
    gamma = acos(z1 / l / cos(beta));
    float x2 = l * (sin(alpha)*sin(beta)*cos(gamma) - cos(alpha)*sin(gamma));
    float y2 = l * (-sin(alpha)*sin(gamma) - cos(alpha)*sin(beta)*cos(alpha));
    float z2 = l * (cos(beta)*cos(gamma));

    line.line.p1 = line.anchor1 + tf2::Vector3(x1, y1, z1);
    line.line.p2 = line.anchor2 + tf2::Vector3(x2, y2, z2);

    cout << "---" << endl;
    cout << "z1 / l / cos(beta) = " << z1 / l / cos(beta) << endl;
    cout << "Length first rope: " << tf2::tf2Distance(line.line.p1, line.anchor1) << endl;
    cout << "Length second rope: " << tf2::tf2Distance(line.line.p2, line.anchor2) << endl;
    cout << tf2::tf2Distance(line.line.p1, line.line.p2) << endl;
    tf2::Vector3 ff = line.line.p1 - line.line.p2;
    cout << ff[0] << " ; " << ff[1] << " ; " << ff[2] << endl;

    // line.line.p1[0] = 0;
    // line.line.p1[1] = i * offset_lines_;
    // line.line.p1[2] = 0;

    // line.line.p2[0] = length_lines_;
    // line.line.p2[1] = i * offset_lines_;
    // line.line.p2[2] = 0;

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
