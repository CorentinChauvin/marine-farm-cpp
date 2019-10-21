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
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <csignal>
#include <string>
#include <iostream>

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
  tf_listener_(tf_buffer_)
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

  if (algae_msg_received_)
    update_coll_world();

  if (get_camera_tf()) {
    // Publish camera field of view on Rviz
    MarkerArgs args;
    args.stamp = ros::Time::now();
    args.frame_id = camera_frame_;
    args.ns = "FieldOfView";
    args.duration = ros::Duration(1/camera_freq_);
    args.color.r = fov_color_[0];
    args.color.g = fov_color_[1];
    args.color.b = fov_color_[2];
    args.color.a = fov_color_[3];

    publish_fov(args);
  }
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


void CameraNodelet::update_coll_world()
{
  // Clean collision world (FIXME: probably not optimal)
  unsigned int n = coll_bodies_.size();

  for (unsigned int k = 0; k < n; k++) {
    // cout << ((coll_bodies_[k]->getTransform()).getPosition()).y << endl;
    coll_world_.destroyCollisionBody(coll_bodies_[k]);
  }

  // Add bodies to collision world
  n = last_algae_msg_->algae.size();
  coll_bodies_.resize(n);
  coll_shapes_.resize(n);

  for (unsigned int k = 0; k < n; k++) {
    const farm_simulator::Alga *al = &last_algae_msg_->algae[k];

    // Creating a collision body
    rp3d::Vector3 pos(al->position.x, al->position.y, al->position.z);
    rp3d::Quaternion orient(al->orientation.x, al->orientation.y,
      al->orientation.z, al->orientation.w);

    rp3d::Transform transform(pos, orient);
    rp3d::CollisionBody* body = coll_world_.createCollisionBody(transform);
    coll_bodies_[k] = body;

    // Adding a collision shape to the body
    rp3d::Vector3 half_extents(al->dimensions.x/2, al->dimensions.y/2,
      al->dimensions.z/2);
    coll_shapes_[k] = unique_ptr<rp3d::BoxShape>(new rp3d::BoxShape(half_extents));
    body->addCollisionShape(coll_shapes_[k].get(), rp3d::Transform::identity());
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

  camera_tf_ = tf.transform;
  return true;
}


void CameraNodelet::publish_fov(MarkerArgs args)
{
  visualization_msgs::Marker marker = rviz_marker_triangles(args);
  marker.points.reserve(18);
  marker.colors.resize(6, args.color);

  // Form the limit points of the field of view
  float a = fov_distance_ / sqrt(
    pow(sensor_width_/2, 2) + pow(sensor_height_/2, 2) + pow(focal_length_, 2)
  );
  float m_x[4] = {-1, 1, 1, -1};  // multiplicators to form the points
  float m_y[4] = {-1, -1, 1, 1};
  tf2::Vector3 origin(0, 0, -focal_length_);
  vector<tf2::Vector3> p(4);

  for (int k = 0; k < 4; k++) {
    p[k] = origin + tf2::Vector3(m_x[k]*a*sensor_width_/2,
                                 m_y[k]*a*sensor_height_/2,
                                 a*focal_length_);
  }

  // Create the corresponding triangles
  geometry_msgs::Point point;
  marker.points.emplace_back(tf2::toMsg(origin, point));
  marker.points.emplace_back(tf2::toMsg(p[0], point));
  marker.points.emplace_back(tf2::toMsg(p[1], point));

  marker.points.emplace_back(tf2::toMsg(origin, point));
  marker.points.emplace_back(tf2::toMsg(p[1], point));
  marker.points.emplace_back(tf2::toMsg(p[2], point));

  marker.points.emplace_back(tf2::toMsg(origin, point));
  marker.points.emplace_back(tf2::toMsg(p[2], point));
  marker.points.emplace_back(tf2::toMsg(p[3], point));

  marker.points.emplace_back(tf2::toMsg(origin, point));
  marker.points.emplace_back(tf2::toMsg(p[3], point));
  marker.points.emplace_back(tf2::toMsg(p[0], point));

  marker.points.emplace_back(tf2::toMsg(p[0], point));
  marker.points.emplace_back(tf2::toMsg(p[1], point));
  marker.points.emplace_back(tf2::toMsg(p[2], point));
  marker.points.emplace_back(tf2::toMsg(p[0], point));
  marker.points.emplace_back(tf2::toMsg(p[2], point));
  marker.points.emplace_back(tf2::toMsg(p[3], point));

  // Publish the marker
  rviz_pub_.publish(marker);
}


}  // namespace mfcpp
