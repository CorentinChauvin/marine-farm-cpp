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

  // FIXME: to remove
  test_pub_ = nh_.advertise<visualization_msgs::Marker>("debug_markers", 0);


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
    unsigned int b = al->disease_heatmap[0].array.size();
    heatmaps_[k].resize(a, vector<float>(b));

    for (unsigned int i = 0; i < a; i++) {
      for (unsigned int j = 0; j < b; j++) {
        heatmaps_[k][i][j] = al->disease_heatmap[i].array[j];
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
  // Prepare the output
  int n_pxl_h = req.n_pxl_height;
  int n_pxl_w = req.n_pxl_width;
  if (n_pxl_h <= 0 || n_pxl_w <= 0) {
    n_pxl_h = n_pxl_height_;
    n_pxl_w = n_pxl_width_;
  }

  // FIXME: to remove
  n_pxl_h = 5;  //<<<<<<<<<<<<<<<<
  n_pxl_w = 10;

  int n_pixels = n_pxl_h * n_pxl_w;
  int nbr_poses = req.pose_array.poses.size();
  res.results.resize(nbr_poses);

  for (int k = 0; k < nbr_poses; k++) {
    res.results[k].x.reserve(n_pixels);
    res.results[k].y.reserve(n_pixels);
    res.results[k].z.reserve(n_pixels);
  }

  // Transform the poses in camera frame
  if (!world_init_ || !get_tf()) {
    res.is_success = false;
    return true;
  }

  vector<geometry_msgs::Pose> poses(nbr_poses);

  for (unsigned int k = 0; k < nbr_poses; k++) {
    geometry_msgs::Pose transf_pose;
    tf2::doTransform(req.pose_array.poses[k], transf_pose, camera_robot_tf_);

    poses[k] = transf_pose;
  }

  // Creating a collision body for field of view at all poses
  rp3d::CollisionBody* body = coll_world_.createCollisionBody(rp3d::Transform::identity());
  unique_ptr<rp3d::BoxShape> shape;
  multi_fov_body(poses, body, shape);

  // Selects algae that are in field of view of the camera
  coll_mutex_.lock();
  overlap_fov(body);

  // Raycast for each pose
  for (int k = 0; k < nbr_poses; k++) {
    geometry_msgs::TransformStamped robot_vp_tf = pose_to_tf(req.pose_array.poses[k]);  // transform from robot to view point
    tf2::Vector3 origin(poses[k].position.x, poses[k].position.y, poses[k].position.z);
    tf2::Vector3 hit_pt;
    int alga_idx;

    // Check the four corners
    int x_corner[4] = {0, n_pxl_h-1, n_pxl_h-1, 0};  // position of the 4 corners in height direction
    int y_corner[4] = {0, 0, n_pxl_w-1, n_pxl_w-1};  // position of the 4 corners in width direction
    bool hit_all_corners = true;

    for (int l = 0; l < 4; l++) {
      // Transform aim point into camera frame
      tf2::Vector3 aim_pt1 = get_aim_pt(x_corner[l], y_corner[l], n_pxl_h, n_pxl_w);  // aim point in view point camera frame
      tf2::Vector3 aim_pt2 = apply_transform(aim_pt1, robot_camera_tf_);  // aim point in view point frame
      tf2::Vector3 aim_pt3 = apply_transform(aim_pt2, robot_vp_tf);       // aim point in robot frame
      tf2::Vector3 aim_pt  = apply_transform(aim_pt3, camera_robot_tf_);  // aim point in camera frame

      // Perform raycast
      bool alga_hit = raycast_alga(aim_pt, hit_pt, alga_idx, origin);

      if (alga_hit) {
        tf2::Vector3 out_pt = apply_transform(hit_pt, camera_fixed_tf_);  // transform in camera frame

        res.results[k].x.emplace_back(out_pt.getX());
        res.results[k].y.emplace_back(out_pt.getY());
        res.results[k].z.emplace_back(out_pt.getZ());
      } else {
        hit_all_corners = false;
      }
    }

    // If all corners have been hit, simply interpolate between it.
    // This is not exact since all rays are not parallel, but it's a fair enough
    // approximation.
    if (hit_all_corners) {
      tf2::Vector3 p1(res.results[k].x[0], res.results[k].y[0], res.results[k].z[0]);
      tf2::Vector3 p2(res.results[k].x[1], res.results[k].y[1], res.results[k].z[1]);
      tf2::Vector3 p3(res.results[k].x[2], res.results[k].y[2], res.results[k].z[2]);
      tf2::Vector3 p4(res.results[k].x[3], res.results[k].y[3], res.results[k].z[3]);

      tf2::Vector3 dx = p2 - p1;
      tf2::Vector3 dy = p4 - p1;
      dx /= n_pxl_h - 1;
      dy /= n_pxl_w - 1;

      for (int i = 0; i < n_pxl_h; i++) {
        for (int j = 0; j < n_pxl_w; j++) {
          if ((i != 0 && i != n_pxl_h-1) || (j != 0 && j != n_pxl_w-1)) {
            tf2::Vector3 p = p1 + i * dx + j * dy;

            res.results[k].x.emplace_back(p.getX());
            res.results[k].y.emplace_back(p.getY());
            res.results[k].z.emplace_back(p.getZ());
          }
        }
      }
    }

    // Otherwise, just raycast through all pixels
    else {
      for (int i = 0; i < n_pxl_h; i++) {
        for (int j = 0; j < n_pxl_w; j++) {
          if ((i != 0 && i != n_pxl_h-1) || (j != 0 && j != n_pxl_w-1)) {
            // Transform aim point into camera frame
            tf2::Vector3 aim_pt1 = get_aim_pt(i, j, n_pxl_h, n_pxl_w);  // aim point in view point camera frame
            tf2::Vector3 aim_pt2 = apply_transform(aim_pt1, robot_camera_tf_);  // aim point in view point frame
            tf2::Vector3 aim_pt3 = apply_transform(aim_pt2, robot_vp_tf);       // aim point in robot frame
            tf2::Vector3 aim_pt  = apply_transform(aim_pt3, camera_robot_tf_);  // aim point in camera frame

            // Perform raycast
            bool alga_hit = raycast_alga(aim_pt, hit_pt, alga_idx, origin);

            if (alga_hit) {
              tf2::Vector3 out_pt = apply_transform(hit_pt, camera_fixed_tf_);  // transform in camera frame

              res.results[k].x.emplace_back(out_pt.getX());
              res.results[k].y.emplace_back(out_pt.getY());
              res.results[k].z.emplace_back(out_pt.getZ());
            }
          }
        }
      }
    }
  }

  // FIXME: to remove
  // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // Display the corners in Rviz to check whether it works in weird situations
  visualization_msgs::Marker pts_marker;
  pts_marker.header.stamp = ros::Time::now();
  pts_marker.header.frame_id = camera_frame_;
  pts_marker.ns = "Hit point";
  pts_marker.lifetime = ros::Duration(1/camera_freq_);
  pts_marker.color.r = 0.0;
  pts_marker.color.g = 0.0;
  pts_marker.color.b = 1.0;
  pts_marker.color.a = 1.0;
  pts_marker.type = visualization_msgs::Marker::POINTS;
  pts_marker.action = visualization_msgs::Marker::ADD;
  pts_marker.scale.x = 0.05;
  pts_marker.scale.y = 0.05;
  pts_marker.points.reserve(n_pixels*nbr_poses);
  pts_marker.colors.reserve(n_pixels*nbr_poses);


  for (int k = 0; k < nbr_poses; k++) {
    float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    float g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    float b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

    for (int l = 0; l < res.results[k].x.size(); l++) {
      tf2::Vector3 hit_pt(res.results[k].x[l], res.results[k].y[l], res.results[k].z[l]);

      if (k == 5)
        add_pt_to_marker(pts_marker, hit_pt, r, g, b);
    }
  }

  test_pub_.publish(pts_marker);

  // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

  // Conclude
  coll_world_.destroyCollisionBody(body);
  coll_mutex_.unlock();
  res.is_success = true;

  return true;
}


}  // namespace mfcpp
