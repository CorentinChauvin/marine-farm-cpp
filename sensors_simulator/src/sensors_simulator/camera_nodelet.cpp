/**
 * @file
 *
 * \brief  Definition of a nodelet simulating a camera
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "camera_nodelet.hpp"
#include "farm_simulator/rviz_visualisation.hpp"
#include "farm_simulator/Algae.h"
#include "reactphysics3d.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <csignal>
#include <string>
#include <iostream>

#include <ctime>

using namespace std;

PLUGINLIB_EXPORT_CLASS(mfcpp::CameraNodelet, nodelet::Nodelet)


namespace mfcpp {

/*
 * Definition of static varibles
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
  private_nh_.param<string>("camera_frame", camera_frame_, "camera");
  private_nh_.param<vector<float>>("fov_color", fov_color_, vector<float>(4, 1.0));
  private_nh_.param<float>("focal_length", focal_length_, 0.0028);
  private_nh_.param<float>("sensor_width", sensor_width_, 0.0064);
  private_nh_.param<float>("sensor_height", sensor_height_, 0.00384);
  private_nh_.param<float>("fov_distance", fov_distance_, 2.0);

  // ROS subscribers
  algae_sub_ = private_nh_.subscribe<farm_simulator::Algae>("algae", 1,
    boost::bind(&CameraNodelet::algae_cb, this, _1));

  // ROS publishers
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
  clock_t begin = clock();
  if (!ros::ok() || ros::isShuttingDown() || b_sigint_)
    return;

  if (!world_init_ && algae_msg_received_) {
    init_coll_world();
    world_init_ = true;
  } else if (algae_msg_received_) {
    update_algae();
    algae_msg_received_ = false;
  }

  cout <<  "Update time: " << double(clock() - begin) / CLOCKS_PER_SEC << endl;
  begin = clock();
  if (world_init_ && get_camera_tf()) {
    publish_rviz_fov();
    publish_output();
  }

  cout <<  "Collision time: " << double(clock() - begin) / CLOCKS_PER_SEC << endl;
}


void CameraNodelet::sigint_handler(int s) {
  b_sigint_ = 1;
  main_timer_.stop();
}


void CameraNodelet::algae_cb(const farm_simulator::AlgaeConstPtr msg)
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

  for (unsigned int k = 0; k < n; k++) {
    const farm_simulator::Alga *al = &last_algae_msg_->algae[k];

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
  }
}


void CameraNodelet::update_algae()
{
  unsigned int n = last_algae_msg_->algae.size();

  for (unsigned int k = 0; k < n; k++) {
    const farm_simulator::Alga *al = &last_algae_msg_->algae[k];

    // Creating a collision body
    rp3d::Vector3 pos(al->position.x, al->position.y, al->position.z);
    rp3d::Quaternion orient(al->orientation.x, al->orientation.y,
      al->orientation.z, al->orientation.w);

    rp3d::Transform transform(pos, orient);
    algae_bodies_[k]->setTransform(transform);
  }
}


bool CameraNodelet::get_camera_tf()
{
  geometry_msgs::TransformStamped tf;

  try {
    tf = tf_buffer_.lookupTransform(fixed_frame_, camera_frame_, ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    NODELET_WARN("%s",ex.what());
    return false;
  }

  camera_tf_ = tf;
  return true;
}


}  // namespace mfcpp
