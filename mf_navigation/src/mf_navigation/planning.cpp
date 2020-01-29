/**
 * @file
 *
 * \brief  Definition of path planning functions
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "planning_nodelet.hpp"
#include "mf_common/common.hpp"
#include "mf_common/spline.hpp"
#include "mf_sensors_simulator/MultiPoses.h"
#include "mf_mapping/UpdateGP.h"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>
#include <iostream>

using namespace std;
using Eigen::Vector3f;


namespace mfcpp {

vector<mf_mapping::Float32Array> PlanningNodelet::vector2D_to_array(
  const vector<vector<float>> &vector2D)
{
  int n = vector2D.size();
  vector<mf_mapping::Float32Array> out(n);

  for (int k = 0; k < n; k++) {
    out[k].data = vector2D[k];
  }

  return out;
}


vector<vector<float>> PlanningNodelet::array_to_vector2D(
  const vector<mf_mapping::Float32Array> &array)
{
  int n = array.size();
  vector<vector<float>> out(n);

  for (int k = 0; k < n; k++) {
    out[k] = array[k].data;
  }

  return out;
}


vector<vector<float>> PlanningNodelet::array_to_vector2D(
  const mf_mapping::Array2D &array)
{
  int n = array.data.size();
  vector<vector<float>> out(n);

  for (int k = 0; k < n; k++) {
    out[k] = array.data[k].data;
  }

  return out;
}


void PlanningNodelet::generate_cart_lattice(float max_lat_angle, float max_elev_angle,
  float horizon, float resolution, std::vector<geometry_msgs::Pose> &lattice)
{
  int n_x = horizon / resolution + 1;  // size of the lattice in x direction
  int n_y = horizon * sin(max_lat_angle) / resolution;   // half size in y direction
  int n_z = horizon * sin(max_elev_angle) / resolution;  // half size in z direction
  unsigned int n_lattice = n_x * (2*n_y + 1) * (2*n_z + 1);      // total size of the lattice

  lattice.resize(0);
  lattice.reserve(n_lattice);
  float x = 0;

  for (int i = 0; i < n_x; i++) {
    float y = -n_y * resolution;

    for (int j = -n_y; j <= n_y; j++) {
      float z = -n_z * resolution;

      for (int k = -n_z; k <= n_z; k++) {
        // Compute point angles with respect to the x axis
        float lat_angle = atan2(y, x);
        float elev_angle = atan2(z, x);

        if (abs(lat_angle) <= max_lat_angle && abs(elev_angle) <= max_elev_angle) {
          geometry_msgs::Pose pose;
          pose.position.x = x;
          pose.position.y = y;
          pose.position.z = z;

          // Account for rotation needed to reach the waypoint
          tf2::Quaternion q_orig, q_rot, q_new;
          q_orig.setRPY(0, 0, 0);
          q_rot.setRPY(0.0, -elev_angle, lat_angle);

          q_new = q_rot * q_orig;
          q_new.normalize();
          tf2::convert(q_new, pose.orientation);

          lattice.emplace_back(pose);
        }

        z += resolution;
      }

      y += resolution;
    }

    x += resolution;
  }
}


void PlanningNodelet::generate_lattice(std::vector<geometry_msgs::Pose> &lattice)
{
  float duration = plan_horizon_/plan_speed_;
  int n_horiz = 2*lattice_size_horiz_ + 1;
  int n_vert = 2*lattice_size_vert_ + 1;
  lattice.resize(n_horiz*n_vert);
  int counter = 0;

  for (int k = -lattice_size_horiz_; k <= lattice_size_horiz_; k++) {
    for (int l = -lattice_size_vert_; l <= lattice_size_vert_; l++) {

      // Apply a constant command during a fixed duration
      float prop_speed = robot_model_.steady_propeller_speed(plan_speed_);
      float horiz_angle = max_lat_rudder_ * k / max(lattice_size_horiz_, 1);
      float vert_angle = max_elev_rudder_ * l / max(lattice_size_vert_, 1);

      RobotModel::state_type state = state_;
      RobotModel::input_type input = {prop_speed, horiz_angle, vert_angle, 0};
      robot_model_.integrate(state, input, 0.0, duration, duration/10);

      // Transform the predicted pose from ocean frame to robot frame
      geometry_msgs::Pose pose;
      pose.position.x = state[0];
      pose.position.y = state[1];
      pose.position.z = state[2];
      to_quaternion(state[3], state[4], state[5], pose.orientation);

      tf2::doTransform(pose, lattice[counter], robot_ocean_tf_);
      counter++;
    }
  }
}


std::vector<geometry_msgs::Pose> PlanningNodelet::filter_lattice(
  const std::vector<geometry_msgs::Pose> &lattice_in)
{
  vector<geometry_msgs::Pose> lattice(0);
  lattice.reserve(lattice_in.size());

  for (int k = 0; k < lattice_in.size(); k++) {
    // Transform point in wall frame
    geometry_msgs::Pose p;
    tf2::doTransform(lattice_in[k], p, wall_robot_tf_);

    // Check bounds
    if (p.position.z >= bnd_wall_dist_[0] && p.position.z <= bnd_wall_dist_[1]
      && p.position.x >= bnd_depth_[0] && p.position.x <= bnd_depth_[1]) {
      lattice.emplace_back(lattice_in[k]);
    }
  }

  return lattice;
}


bool PlanningNodelet::compute_lattice_gp(
  vector<vector<float>> &cov_diag,
  vector<vector<float>> &camera_pts_x,
  vector<vector<float>> &camera_pts_y,
  vector<vector<float>> &camera_pts_z)
{
  // Compute the corresponding camera orientation for each viewpoint
  int size_lattice = lattice_.size();
  std::vector<geometry_msgs::Pose> lattice(size_lattice);  // lattice of camera orientations

  for (int k = 0; k < size_lattice; k++) {
    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::convert(lattice_[k].orientation, q_orig);
    q_rot.setRPY(M_PI_2, 0.0, 0.0);

    q_new = q_orig * q_rot;
    q_new.normalize();
    tf2::convert(q_new, lattice[k].orientation);
  }

  // Transform the orientation in camera frame
  geometry_msgs::TransformStamped camera_robot_tf;

  try {
    camera_robot_tf = tf_buffer_.lookupTransform(camera_frame_, robot_frame_, ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    NODELET_WARN("[planning_nodelet] %s", ex.what());
    return false;
  }

  for (int k = 0; k < size_lattice; k++) {
    geometry_msgs::Pose transf_pose;
    tf2::doTransform(lattice[k], transf_pose, camera_robot_tf);

    lattice[k] = transf_pose;
  }

  // Get camera measurement for each viewpoint of the lattice
  mf_sensors_simulator::MultiPoses camera_srv;
  camera_srv.request.pose_array.header.frame_id = robot_frame_;
  camera_srv.request.pose_array.poses = lattice;
  camera_srv.request.n_pxl_height = camera_height_;
  camera_srv.request.n_pxl_width = camera_width_;

  if (ray_multi_client_.call(camera_srv)) {
    if (!camera_srv.response.is_success) {
      NODELET_WARN("[planning_nodelet] Call to raycast_multi service resulted didn't give output ");
      return false;
    }
  } else {
    NODELET_WARN("[planning_nodelet] Failed to call raycast_multi service");
    return false;
  }

  // Store camera hit points
  camera_pts_x.resize(size_lattice);
  camera_pts_y.resize(size_lattice);
  camera_pts_z.resize(size_lattice);

  for (int k = 0; k < size_lattice; k++) {
    camera_pts_x[k] = camera_srv.response.results[k].x;
    camera_pts_y[k] = camera_srv.response.results[k].y;
    camera_pts_z[k] = camera_srv.response.results[k].z;
  }

  // Update the GP covariance for each viewpoint
  mf_mapping::UpdateGP gp_srv;
  gp_srv.request.use_internal_mean = true;
  gp_srv.request.use_internal_cov = true;
  gp_srv.request.update_mean = false;
  gp_srv.request.return_cov_diag = true;
  gp_srv.request.eval_gp = false;

  gp_srv.request.meas.resize(size_lattice);
  for (int k = 0; k < size_lattice; k++) {
    gp_srv.request.meas[k] = camera_srv.response.results[k];
  }

  if (update_gp_client_.call(gp_srv)) {
    for (int k = 0; k < size_lattice; k++) {
      cov_diag[k] = gp_srv.response.new_cov_diag[k].data;
    }
  } else {
    NODELET_WARN("[planning_nodelet] Failed to call update_gp service");
    return false;
  }

  return true;
}


nav_msgs::Path PlanningNodelet::straight_line_path(const geometry_msgs::Pose &start,
  const geometry_msgs::Pose &end, float resolution)
{
  // Convert input
  tf2::Vector3 p1, p2;
  tf2::convert(start.position, p1);
  tf2::convert(end.position, p2);

  tf2::Quaternion q1, q2;
  tf2::convert(start.orientation, q1);
  tf2::convert(end.orientation, q2);

  // Prepare interpolation
  nav_msgs::Path path;
  float d = tf2::tf2Distance(p1, p2);
  int n = d/resolution;

  path.header.frame_id = ocean_frame_;
  path.header.stamp = ros::Time::now();
  path.poses.resize(n + 1);

  // Interpolation
  for (int k = 0; k <= n; k++) {
    geometry_msgs::PoseStamped new_pose;
    tf2::toMsg(   p1 + (p2-p1) * (float(k)/n),              new_pose.pose.position);
    tf2::convert((q1 + (q2-q1) * (float(k)/n)).normalize(), new_pose.pose.orientation);

    path.poses[k] = new_pose;
  }

  return path;
}


nav_msgs::Path PlanningNodelet::spline_path(const geometry_msgs::Pose &start,
  const geometry_msgs::Pose &end, float resolution, float speed)
{
  vector<Eigen::Vector3f> p(2);  // positions of the start and end poses
  vector<Eigen::Vector3f> o(2);  // orientations of the start and end poses

  p[0] << start.position.x, start.position.y, start.position.z;
  p[1] << end.position.x, end.position.y, end.position.z;
  double roll1, roll2, pitch1, pitch2, yaw1, yaw2;
  to_euler(start.orientation, roll1, pitch1, yaw1);
  to_euler(end.orientation, roll2, pitch2, yaw2);
  o[0] << cos(yaw1), sin(yaw1), sin(pitch1);
  o[1] << cos(yaw2), sin(yaw2), sin(pitch2);

  Spline spline(p, o, speed);

  nav_msgs::Path path;
  path.poses.resize(0);
  path.header.frame_id = ocean_frame_;
  bool last_reached = false;
  float t = 0;

  while (!last_reached) {
    Eigen::Vector3f p;
    Eigen::Vector3f o;

    spline.evaluate(t, p, o, last_reached);
    o = o / o.norm();

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = p(0);
    pose.pose.position.y = p(1);
    pose.pose.position.z = p(2);
    to_quaternion(0.0, -asin(o(2)), atan2(o(1), o(0)), pose.pose.orientation);

    path.poses.emplace_back(pose);
    t += resolution / speed;
  }

  return path;
}


bool PlanningNodelet::plan_trajectory()
{
  // Check for Gaussian Process received
  int size_gp = last_gp_cov_.size();

  if (last_gp_cov_.size() == 0 || last_gp_mean_.size() == 0)
    return false;

  // Generate a lattice of possible waypoints (in robot frame)
  vector<geometry_msgs::Pose> lattice_tmp;

  if (cart_lattice_) {
    float lat_turn_radius  = robot_model_.lat_turn_radius(plan_speed_, max_lat_rudder_);
    float elev_turn_radius = robot_model_.elev_turn_radius(plan_speed_, max_lat_rudder_);

    float max_lat_angle  = plan_horizon_ / (2 * lat_turn_radius);
    float max_elev_angle = plan_horizon_ / (2 * elev_turn_radius);

    if (!horiz_motion_) max_lat_angle = 0;
    if (!vert_motion_)  max_elev_angle = 0;

    generate_cart_lattice(max_lat_angle, max_elev_angle, plan_horizon_, lattice_res_, lattice_tmp);
  } else {
    generate_lattice(lattice_tmp);
  }

  lattice_ = filter_lattice(lattice_tmp);

  if (lattice_.size() == 0) {
    NODELET_WARN("[planning_nodelet] No valid waypoint found");
    return false;
  }

  // Update the GP covariance for each viewpoint
  int size_lattice = lattice_.size();
  vector<vector<float>> cov_diag(size_lattice);  // diagonal of the updated GP cov for each view point
  vector<vector<float>> camera_pts_x, camera_pts_y, camera_pts_z;  // camera hit points

  bool ret = compute_lattice_gp(cov_diag, camera_pts_x, camera_pts_y, camera_pts_z);

  if (!ret)
    return false;

  // Compute information gain and select best viewpoint
  vector<float> info_gain(size_lattice, 0.0);

  for (int k = 1; k < size_lattice; k++) {
    for (int l = 0; l < size_gp; l++) {
      float cov_diff = last_gp_cov_[l][l] - cov_diag[k][l];
      float weight = 100 * (1/(1 + exp(-gp_weight_*(last_gp_mean_[l] - 0.5))));
      info_gain[k] += weight * cov_diff;
    }
  }

  selected_vp_ = std::max_element(info_gain.begin(), info_gain.end()) - info_gain.begin();

  x_hit_pt_sel_ = camera_pts_x[selected_vp_];
  y_hit_pt_sel_ = camera_pts_y[selected_vp_];
  z_hit_pt_sel_ = camera_pts_z[selected_vp_];

  // Generate a spline trajectory (in ocean frame) to go to the selected point
  nav_msgs::Path path;
  geometry_msgs::Pose current_pose, selected_pose;
  current_pose = tf_to_pose(ocean_robot_tf_);
  tf2::doTransform(lattice_[selected_vp_], selected_pose, ocean_robot_tf_);

  if (linear_path_)
    path = straight_line_path(current_pose, selected_pose, path_res_);
  else
    path = spline_path(current_pose, selected_pose, path_res_, plan_speed_);

  path_pub_.publish(path);
}


}  // namespace mfcpp
