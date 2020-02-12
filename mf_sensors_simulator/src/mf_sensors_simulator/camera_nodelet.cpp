/**
 * @file
 *
 * \brief  Definition of a nodelet simulating a camera
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "camera_nodelet.hpp"
#include "mf_sensors_simulator/CameraOutput.h"
#include "mf_sensors_simulator/MultiPoses.h"
#include "mf_farm_simulator/rviz_visualisation.hpp"
#include "mf_farm_simulator/Algae.h"
#include "reactphysics3d.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <csignal>
#include <string>
#include <vector>
#include <iostream>

using namespace std;

PLUGINLIB_EXPORT_CLASS(mfcpp::CameraNodelet, nodelet::Nodelet)


namespace mfcpp {

/*
 * Definition of static variables
 */
sig_atomic_t volatile CameraNodelet::b_sigint_ = 0;
ros::Timer CameraNodelet::main_timer_ = ros::Timer();

/*
 * Definition of member functions
 */
CameraNodelet::CameraNodelet():
  tf_listener_(tf_buffer_),
  raycast_cb_(this),
  overlap_cb_(this)
{

}

CameraNodelet::~CameraNodelet() {}


void CameraNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Catch SIGINT (Ctrl+C) stop signal
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = CameraNodelet::sigint_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // ROS parameters
  private_nh_.param<float>("camera_freq", camera_freq_, 1.0);
  private_nh_.param<string>("fixed_frame", fixed_frame_, "ocean");
  private_nh_.param<string>("robot_frame", robot_frame_, "base_link");
  private_nh_.param<string>("camera_frame", camera_frame_, "camera");
  private_nh_.param<vector<float>>("fov_color", fov_color_, vector<float>(4, 1.0));
  private_nh_.param<float>("focal_length", focal_length_, 0.0028);
  private_nh_.param<float>("sensor_width", sensor_width_, 0.0064);
  private_nh_.param<float>("sensor_height", sensor_height_, 0.00384);
  private_nh_.param<float>("fov_distance", fov_distance_, 2.0);
  private_nh_.param<int>("n_pxl_height", n_pxl_height_, 480);
  private_nh_.param<int>("n_pxl_width", n_pxl_width_, 800);

  // ROS subscribers
  algae_sub_ = private_nh_.subscribe<mf_farm_simulator::Algae>("algae", 1,
    boost::bind(&CameraNodelet::algae_cb, this, _1));

  // ROS services
  ray_multi_serv_ = nh_.advertiseService("raycast_multi", &CameraNodelet::ray_multi_cb, this);

  // ROS publishers
  out_pub_ = nh_.advertise<mf_sensors_simulator::CameraOutput>("camera_out", 0);
  rviz_pub_ = nh_.advertise<visualization_msgs::Marker>("camera_markers", 0);

  // Other parameters
  algae_msg_received_ = false;
  world_init_ = false;

  // Initialise collision world
  rp3d::CollisionWorld coll_world_(world_settings_);


  // Main loop
  main_timer_ = private_nh_.createTimer(
    ros::Duration(1/camera_freq_), &CameraNodelet::main_cb, this
  );
}


void CameraNodelet::main_cb(const ros::TimerEvent &timer_event)
{
  if (!ros::ok() || ros::isShuttingDown() || b_sigint_)
    return;

  if (!world_init_ && algae_msg_received_) {
    init_coll_world();
    world_init_ = true;
  } else if (algae_msg_received_) {
    update_algae();
    algae_msg_received_ = false;
  }

  if (world_init_ && get_tf()) {
    publish_rviz_fov();
    publish_output();
  }
}


void CameraNodelet::sigint_handler(int s) {
  b_sigint_ = 1;
  main_timer_.stop();

  raise(SIGTERM);
}


void CameraNodelet::algae_cb(const mf_farm_simulator::AlgaeConstPtr msg)
{
  last_algae_msg_ = msg;
  algae_msg_received_ = true;
}


void CameraNodelet::init_coll_world()
{
  // Add camera FOV
  fov_body_ = coll_world_.createCollisionBody(rp3d::Transform::identity());

  float x = sensor_width_ * fov_distance_
          / (2 * sqrt(pow(focal_length_, 2) + pow(sensor_width_, 2)/4));
  float y = sensor_width_ * fov_distance_
          / (2 * sqrt(pow(focal_length_, 2) + pow(sensor_height_, 2)/4));

  rp3d::Vector3 fov_half_extents(x, y, fov_distance_/2);
  fov_shape_ = unique_ptr<rp3d::BoxShape>(new rp3d::BoxShape(fov_half_extents));

  rp3d::Vector3 fov_pos(0, 0, fov_distance_/2);
  rp3d::Quaternion fov_orient(0, 0, 0, 1);
  rp3d::Transform fov_transform(fov_pos, fov_orient);
  fov_body_->addCollisionShape(fov_shape_.get(), fov_transform);

  // Add algae
  unsigned int n = last_algae_msg_->algae.size();
  algae_bodies_.resize(n);
  algae_shapes_.resize(n);
  heatmaps_.resize(n);

  for (unsigned int k = 0; k < n; k++) {
    const mf_farm_simulator::Alga *al = &last_algae_msg_->algae[k];

    // Creating a collision body
    rp3d::Vector3 pos(al->position.x, al->position.y, al->position.z);
    rp3d::Quaternion orient(al->orientation.x, al->orientation.y,
      al->orientation.z, al->orientation.w);

    rp3d::Transform transform(pos, orient);
    rp3d::CollisionBody* body = coll_world_.createCollisionBody(transform);
    algae_bodies_[k] = body;

    // Adding a collision shape to the body
    rp3d::Vector3 half_extents(al->dimensions.x/2, al->dimensions.y/2,
      al->dimensions.z/2);
    algae_shapes_[k] = unique_ptr<rp3d::BoxShape>(new rp3d::BoxShape(half_extents));
    body->addCollisionShape(algae_shapes_[k].get(), rp3d::Transform::identity());

    // Storing disease heatmap
    unsigned int a = al->disease_heatmap.size();
    unsigned int b = al->disease_heatmap[0].data.size();
    heatmaps_[k].resize(a, vector<float>(b));

    for (unsigned int i = 0; i < a; i++) {
      for (unsigned int j = 0; j < b; j++) {
        heatmaps_[k][i][j] = al->disease_heatmap[i].data[j];
      }
    }
  }
}


void CameraNodelet::update_algae()
{
  unsigned int n = last_algae_msg_->algae.size();

  for (unsigned int k = 0; k < n; k++) {
    const mf_farm_simulator::Alga *al = &last_algae_msg_->algae[k];

    // Creating a collision body
    rp3d::Vector3 pos(al->position.x, al->position.y, al->position.z);
    rp3d::Quaternion orient(al->orientation.x, al->orientation.y,
      al->orientation.z, al->orientation.w);

    rp3d::Transform transform(pos, orient);
    algae_bodies_[k]->setTransform(transform);
  }
}


bool CameraNodelet::get_tf()
{
  geometry_msgs::TransformStamped tf1, tf2, tf3, tf4;

  try {
    tf1 = tf_buffer_.lookupTransform(fixed_frame_, camera_frame_, ros::Time(0));
    tf2 = tf_buffer_.lookupTransform(camera_frame_, fixed_frame_, ros::Time(0));
    tf3 = tf_buffer_.lookupTransform(camera_frame_, robot_frame_, ros::Time(0));
    tf4 = tf_buffer_.lookupTransform(robot_frame_, camera_frame_, ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    NODELET_WARN("[camera_nodelet] %s",ex.what());
    return false;
  }

  fixed_camera_tf_ = tf1;
  camera_fixed_tf_ = tf2;
  camera_robot_tf_ = tf3;
  robot_camera_tf_ = tf4;
  return true;
}


bool CameraNodelet::ray_multi_cb(mf_sensors_simulator::MultiPoses::Request &req,
  mf_sensors_simulator::MultiPoses::Response &res)
{
  // Check for initialisation
  if (!world_init_ || !get_tf()) {
    res.is_success = false;
    return true;
  }

  // Prepare the output
  int n_pxl_h = req.n_pxl_height;
  int n_pxl_w = req.n_pxl_width;
  if (n_pxl_h <= 0 || n_pxl_w <= 0) {
    n_pxl_h = n_pxl_height_;
    n_pxl_w = n_pxl_width_;
  }

  int nbr_poses = req.pose_array.poses.size();
  res.results.resize(nbr_poses);

  // Creating a collision body for field of view at all poses
  rp3d::CollisionBody* body = coll_world_.createCollisionBody(rp3d::Transform::identity());
  unique_ptr<rp3d::BoxShape> shape;
  multi_fov_body(req.pose_array.poses, body, shape);

  // Selects algae that are in field of view of the camera
  coll_mutex_.lock();
  overlap_fov(body);

  // Raycast all pixels for each pose
  for (int k = 0; k < nbr_poses; k++) {
    raycast_wall(req.pose_array.poses[k], n_pxl_h, n_pxl_w, res.results[k]);
  }

  // Conclude
  coll_world_.destroyCollisionBody(body);
  coll_mutex_.unlock();
  res.is_success = true;

  return true;
}


}  // namespace mfcpp
