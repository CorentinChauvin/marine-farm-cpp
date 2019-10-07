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

#include <ctime>



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
  private_nh_.param<int>("height_disease_heatmap", height_disease_heatmap_, 30);
  private_nh_.param<int>("width_disease_heatmap", width_disease_heatmap_, 10);
  private_nh_.param<float>("disease_ratio", disease_ratio_, 0.5);


  // Other variables

  // ROS publishers
  rviz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("farm", 0 );

  // Create algae lines
  clock_t begin = clock();

  init_algae_lines();

  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

  cout << "After init(): " << elapsed_secs << endl;


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
    phi_lines_ = config.phi_lines;
    theta_lines_ = config.theta_lines;
    bnd_phi_lines_ = config.bnd_phi_lines;
    bnd_theta_lines_ = config.bnd_theta_lines;
    bnd_gamma_lines_ = config.bnd_gamma_lines;

    init_algae_lines();
  } else
    reconfigure_initialised_ = true;
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

    perlin_.configure(height_disease_heatmap_, width_disease_heatmap_, 3, 3);
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
            alga.disease_heatmap[i][j] = perlin_.accentuate(value);
          }
        }

        // Add the alga
        line.algae.emplace_back(alga);
      }
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
  args.color.r = 0.8;
  args.color.g = 0.8;
  args.color.b = 0.8;
  args.color.a = 1.0;

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

  // Add buoys
  args.ns = "buoys";
  visualization_msgs::Marker buoys_marker = rviz_marker_spheres(buoys_diameter_, args);
  buoys_marker.points.reserve(nbr_lines_ * nbr_buoys_);

  for (unsigned int i = 0; i < nbr_lines_; i++) {
    const AlgaeLine *al = &algae_lines_[i];  // for convenience
    geometry_msgs::Point point;
    tf2::Vector3 X1 = al->floating_rope.p1;
    tf2::Vector3 X2 = al->floating_rope.p2;
    unsigned int n = al->nbr_buoys;

    for (unsigned int k = 0; k < n; k++) {
      tf2::Vector3 X = X1 + float(k)/(n-1) * (X2-X1);
      buoys_marker.points.emplace_back(tf2::toMsg(X, point));
    }
  }

  markers.markers.emplace_back(buoys_marker);

  // Add ropes
  args.ns = "ropes";
  visualization_msgs::Marker line_marker = rviz_marker_line(thickness_ropes_, args);
  line_marker.points.reserve(3 * nbr_lines_);

  for (unsigned int i = 0; i < nbr_lines_; i++) {
    const AlgaeLine *al = &algae_lines_[i];
    geometry_msgs::Point point;

    line_marker.points.emplace_back(tf2::toMsg(al->anchor1, point));
    line_marker.points.emplace_back(tf2::toMsg(al->floating_rope.p1, point));
    line_marker.points.emplace_back(tf2::toMsg(al->floating_rope.p2, point));
    line_marker.points.emplace_back(tf2::toMsg(al->anchor2, point));
    line_marker.points.emplace_back(tf2::toMsg(al->floating_rope.p1, point));
    line_marker.points.emplace_back(tf2::toMsg(al->floating_rope.p2, point));
    line_marker.points.emplace_back(tf2::toMsg(al->line.p1, point));
    line_marker.points.emplace_back(tf2::toMsg(al->line.p2, point));

    // Ropes between the buoys and the algae line
    unsigned int n = al->nbr_buoys;
    tf2::Vector3 X1 = al->floating_rope.p1;
    tf2::Vector3 X2 = al->floating_rope.p2;
    tf2::Vector3 Y1 = al->line.p1;
    tf2::Vector3 Y2 = al->line.p2;

    for (unsigned int k = 1; k < n-1; k++) {
      tf2::Vector3 X = X1 + float(k)/(n-1) * (X2-X1);
      tf2::Vector3 Y = Y1 + float(k)/(n-1) * (Y2-Y1);
      line_marker.points.emplace_back(tf2::toMsg(X, point));
      line_marker.points.emplace_back(tf2::toMsg(Y, point));
    }
  }

  markers.markers.emplace_back(line_marker);

  // Add algae
  args.ns = "algae";
  visualization_msgs::Marker rect_marker = rviz_marker_rect(args);
  rect_marker.points.reserve(2 * nbr_lines_ * nbr_algae_);

  for (unsigned int i = 0; i < nbr_lines_; i++) {
    const AlgaeLine *al = &algae_lines_[i];

    tf2::Vector3 X1 = al->line.p1;
    tf2::Vector3 X2 = al->line.p2;
    tf2::Vector3 z(0, 0, 1);
    tf2::Vector3 y = (X2-X1) / tf2::tf2Distance(X1, X2);
    tf2::Vector3 x = tf2::tf2Cross(y, z);

    for (unsigned int k = 0; k < al->algae.size(); k++) {
      tf2::Vector3 X = al->algae[k].position;
      float psi = al->algae[k].orientation;
      float H = al->algae[k].length;
      float W = al->algae[k].width;

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

  std_msgs::ColorRGBA color;
  color.r = 0.1;
  color.g = 0.8;
  color.b = 0.1;
  color.a = 1.0;
  rect_marker.colors.resize(rect_marker.points.size()/3, color);

  markers.markers.emplace_back(rect_marker);

  // TEST on heatmaps
  float W = 0.20;
  float H = 1;
  int n_W = 10;
  int n_H = 50;
  args.ns = "test";
  visualization_msgs::Marker img_marker = rviz_marker_rect(args);
  img_marker.points.reserve(2*n_W*n_H);
  img_marker.colors.reserve(2*n_W*n_H);
  img_marker.pose.position.x = -2.0;

  for (int i = 0; i < n_W; i++) {
    tf2::Vector3 p1, p2, p3, p4;
    p1[1] = float(i)/n_W*W;
    p2[1] = float(i+1)/n_W*W;
    p3[1] = float(i+1)/n_W*W;
    p4[1] = float(i)/n_W*W;
    p1[0] = 0.0;
    p2[0] = 0.0;
    p3[0] = 0.0;
    p4[0] = 0.0;

    for (int j = 0; j < n_H; j++) {
      p1[2] = float(j)/n_H*H;
      p2[2] = float(j)/n_H*H;
      p3[2] = float(j+1)/n_H*H;
      p4[2] = float(j+1)/n_H*H;

      std_msgs::ColorRGBA color;
      // float perlin_color = perlin_.evaluate((p1[1]+p2[1])/2, (p1[2]+p4[2])/2);
      // float a = 0.5;
      // float value = max((float)a*faded(faded(faded(faded(perlin_color)))), (float)0.0);
      // value = min(value, (float)1);
      //
      // color.r = value;
      // color.g = value;
      // color.b = value;
      color.a = 1;
      img_marker.colors.emplace_back(color);
      img_marker.colors.emplace_back(color);

      geometry_msgs::Point point;
      img_marker.points.emplace_back(tf2::toMsg(p1, point));  // first triangle
      img_marker.points.emplace_back(tf2::toMsg(p2, point));
      img_marker.points.emplace_back(tf2::toMsg(p4, point));

      img_marker.points.emplace_back(tf2::toMsg(p2, point));  // second triangle
      img_marker.points.emplace_back(tf2::toMsg(p3, point));
      img_marker.points.emplace_back(tf2::toMsg(p4, point));
    }
  }
  markers.markers.emplace_back(img_marker);

  // Publish the markers
  pop_marker_ids(markers);
  rviz_pub_.publish(markers);

}


}  // namespace mfcpp
