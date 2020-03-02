/**
 * @file
 *
 * \brief  Definition of path planning functions
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "planning_nodelet.hpp"
#include "lattice.hpp"
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

vector<mf_common::Float32Array> PlanningNodelet::vector2D_to_array(
  const vector<vector<float>> &vector2D)
{
  int n = vector2D.size();
  vector<mf_common::Float32Array> out(n);

  for (int k = 0; k < n; k++) {
    out[k].data = vector2D[k];
  }

  return out;
}


vector<vector<float>> PlanningNodelet::array_to_vector2D(
  const vector<mf_common::Float32Array> &array)
{
  int n = array.size();
  vector<vector<float>> out(n);

  for (int k = 0; k < n; k++) {
    out[k] = array[k].data;
  }

  return out;
}


vector<vector<float>> PlanningNodelet::array_to_vector2D(
  const mf_common::Array2D &array)
{
  int n = array.data.size();
  vector<vector<float>> out(n);

  for (int k = 0; k < n; k++) {
    out[k] = array.data[k].data;
  }

  return out;
}


Eigen::Vector3f PlanningNodelet::get_wall_orientation(float &yaw_wall)
{
  double roll, pitch, yaw;
  to_euler(ocean_robot_tf_.transform.rotation, roll, pitch, yaw);

  yaw_wall = wall_orientation_;

  while (fabs(yaw_wall - yaw) > M_PI_2) {
    if (yaw_wall - yaw > M_PI_2)
      yaw_wall -= M_PI;
    else
      yaw_wall += M_PI;
  }

  return Eigen::Vector3f(cos(yaw_wall), sin(yaw_wall), 0.0);
}


Eigen::Vector3f PlanningNodelet::get_wall_orientation()
{
  float foo;
  return get_wall_orientation(foo);
}


void PlanningNodelet::generate_cart_lattice(float max_lat_angle, float max_elev_angle,
  float horizon, float resolution, Lattice &lattice)
{
  int n_x = horizon / resolution;  // size of the lattice in x direction
  int n_y = horizon * sin(max_lat_angle) / resolution;   // half size in y direction
  int n_z = horizon * sin(max_elev_angle) / resolution;  // half size in z direction
  unsigned int n_lattice = n_x * (2*n_y + 1) * (2*n_z + 1);      // total size of the lattice

  lattice.nodes.clear();
  lattice.nodes.reserve(n_lattice);
  float x = 0;

  for (int i = 0; i < n_x; i++) {
    x += resolution;
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

          lattice.nodes.emplace_back(new LatticeNode());
          lattice.nodes.back()->pose = pose;
        }

        z += resolution;
      }

      y += resolution;
    }
  }
}


void PlanningNodelet::generate_lattice(Lattice &lattice)
{
  float duration = plan_horizon_/plan_speed_;
  int n_horiz = 2*lattice_size_horiz_ + 1;
  int n_vert = 2*lattice_size_vert_ + 1;
  lattice.nodes.clear();
  lattice.nodes.resize(n_horiz*n_vert);
  int counter = 0;

  for (int k = -lattice_size_horiz_; k <= lattice_size_horiz_; k++) {
    for (int l = -lattice_size_vert_; l <= lattice_size_vert_; l++) {
      // Apply a command during a fixed duration
      float prop_speed = robot_model_.steady_propeller_speed(plan_speed_);   // constant speed command
      float vert_angle = max_elev_rudder_ * l / max(lattice_size_vert_, 1);  // constant vertical angle command
      float ref_horiz_angle = max_lat_rudder_ * k / max(lattice_size_horiz_, 1);  // reference horizontal angle

      RobotModel::state_type state = state_;
      int N = 10;
      for (int cnt = 0; cnt < N; cnt++) {
        float t = float(cnt) / N;
        float current_horiz_angle = ref_horiz_angle*(-2*pow(t, 3) + 4*pow(t,2) - 1);  // cubic interpolation for horizontal angle
        RobotModel::input_type input = {prop_speed, current_horiz_angle, vert_angle, 0};

        robot_model_.integrate(state, input, 0.0, duration/N, duration/N/10);
      }

      // Transform the predicted pose from ocean frame to robot frame
      geometry_msgs::Pose pose, lattice_pose;
      pose.position.x = state[0];
      pose.position.y = state[1];
      pose.position.z = state[2];

      float wall_orientation = wall_orientation_;

      while (fabs(wall_orientation - state[5]) > M_PI_2) {
        if (wall_orientation - state[5] > M_PI_2)
          wall_orientation -= M_PI;
        else
          wall_orientation += M_PI;
      }

      to_quaternion(state[3], state[4], wall_orientation, pose.orientation);

      tf2::doTransform(pose, lattice_pose, robot_ocean_tf_);

      lattice.nodes[counter] = std::shared_ptr<LatticeNode>(new LatticeNode());
      lattice.nodes[counter]->pose = lattice_pose;
      counter++;
    }
  }
}


void PlanningNodelet::generate_lattices(
  const RobotModel::state_type &init_state,
  std::vector<Lattice> &lattices)
{
  // Get wall orientation
  float yaw_wall;
  Eigen::Vector3f orientation_wall = get_wall_orientation(yaw_wall);
  geometry_msgs::Quaternion quati;  // orientation in ocean frame
  to_quaternion(0, 0, yaw_wall, quati);

  geometry_msgs::Quaternion orientation_vp;  // orientation of viewpoints in wall frame
  tf2::doTransform(quati, orientation_vp, wall_ocean_tf_);

  // Get robot initial position (in wall frame)
  geometry_msgs::Point p_ocean, p_wall;  // position in ocean and wall frames
  p_ocean.x = init_state[0];
  p_ocean.y = init_state[1];
  p_ocean.z = init_state[2];
  tf2::doTransform(p_ocean, p_wall, wall_ocean_tf_);

  bool backwards = (p_wall.z < 0);  // whether the robot is going backwards or forwards (with respect to the wall)

  // Generate lattices in wall frame
  int nbr_lattices = lattices.size();
  int nbr_nodes = lattice_size_vert_*lattice_size_horiz_;  // number of nodes per lattice
  float y = p_wall.y;
  float dy = plan_horizon_;

  float margin = 0.01;  // margin to prevent the points to be filtered out
  float dx = fabs(bnd_depth_[1] - bnd_depth_[0] - 2*margin) / (lattice_size_vert_-1);  // vertical increment
  float dz = fabs(bnd_wall_dist_[1] - bnd_wall_dist_[0] - 2*margin) / (lattice_size_horiz_-1);  // horizontal increment

  if (backwards) {
    dy = -dy;
    dz = -dz;
  }

  for (int k = 0; k < nbr_lattices; k++) {
    lattices[k].nodes.clear();
    lattices[k].nodes.reserve(nbr_nodes);
    y += dy;
    float x = bnd_depth_[0] + margin/(lattice_size_vert_-1);

    for (int l = 0; l < lattice_size_vert_; l++) {
      float z = bnd_wall_dist_[0] + margin/(lattice_size_horiz_-1);

      if (backwards)
        z = -z;

      for (int m = 0; m < lattice_size_horiz_; m++) {
        // Create a new node to add to the lattice
        shared_ptr<LatticeNode> new_node (new LatticeNode());
        new_node->pose.position.x = x;
        new_node->pose.position.y = y;
        new_node->pose.position.z = z;
        new_node->pose.orientation = orientation_vp;

        lattices[k].nodes.emplace_back(std::move(new_node));

        z += dz;
      }

      x += dx;
    }
  }

  // Transform lattice from wall to robot frame
  for (int k = 0; k < nbr_lattices; k++) {
    for (int l = 0; l < nbr_nodes; l++) {
      geometry_msgs::Pose pose_tmp;
      tf2::doTransform(lattices[k].nodes[l]->pose, pose_tmp, robot_wall_tf_);
      lattices[k].nodes[l]->pose = pose_tmp;
    }
  }
}


void PlanningNodelet::filter_lattice(Lattice &lattice_in, Lattice &lattice_out)
{
  lattice_out.nodes.reserve(lattice_in.nodes.size());

  for (int k = 0; k < lattice_in.nodes.size(); k++) {
    bool position_ok = false;
    bool pitch_ok = false;
    bool orientation_ok = false;

    // Transform point in wall and ocean frames
    geometry_msgs::Pose vp1;  // viewpoint expressed in wall frame
    geometry_msgs::Pose vp2;  // viewpoint expressed in ocean frame
    tf2::doTransform(lattice_in.nodes[k]->pose, vp1, wall_robot_tf_);
    tf2::doTransform(lattice_in.nodes[k]->pose, vp2, ocean_robot_tf_);

    // Check position bounds
    if (fabs(vp1.position.z) >= bnd_wall_dist_[0] && fabs(vp1.position.z) <= bnd_wall_dist_[1]
      && vp1.position.x >= bnd_depth_[0] && vp1.position.x <= bnd_depth_[1]) {
      position_ok = true;
    }

    if (vp1.position.z * wall_robot_tf_.transform.translation.z < 0.0)
      position_ok = false;  // the vp is on the wrong side of the wall

    // Check pitch angle of the viewpoint (not being to vertically inclined)
    double roll, pitch, yaw;
    to_euler(vp2.orientation, roll, pitch, yaw);

    if (abs(pitch) <= bnd_pitch_)
      pitch_ok = true;

    // Check orientation of the viewpoint (not going backwards)
    Eigen::Vector3f orientation_vp;
    get_orientation(roll, pitch, yaw, orientation_vp); // orientation of the viewpoint
    Eigen::Vector3f orientation_wall = get_wall_orientation();  // orientation of the wall

    if (orientation_vp.dot(orientation_wall) >= 0.0)
      orientation_ok = true;

    // Add the viewpoints if it passed all the checks
    if (position_ok && pitch_ok && orientation_ok) {
      lattice_out.nodes.emplace_back(std::move(lattice_in.nodes[k]));
    }
  }
}


void PlanningNodelet::transform_lattices(
  std::vector<Lattice> &lattices,
  const geometry_msgs::TransformStamped &transform)
{
  for (int k = 0; k < lattices.size(); k++) {
    for (int l = 0; l < lattices[k].nodes.size(); l++) {
      geometry_msgs::Pose tmp;
      tf2::doTransform(lattices[k].nodes[l]->pose, tmp, transform);
      lattices[k].nodes[l]->pose = tmp;
    }
  }
}


void PlanningNodelet::add_node(
  const RobotModel::state_type &state,
  const geometry_msgs::TransformStamped &frame_ocean_tf,
  Lattice &lattice)
{
  geometry_msgs::Pose p_ocean, p_frame;
  p_ocean.position.x = state[0];
  p_ocean.position.y = state[1];
  p_ocean.position.z = state[2];
  to_quaternion(state[3], state[4], state[5], p_ocean.orientation);
  tf2::doTransform(p_ocean, p_frame, frame_ocean_tf);

  shared_ptr<LatticeNode> new_node (new LatticeNode());
  new_node->pose = p_frame;
  new_node->speed = state[6];
  lattice.nodes.emplace_back(std::move(new_node));
}


void PlanningNodelet::connect_lattices(
  const RobotModel::state_type &init_state,
  std::vector<Lattice> &lattices_in,
  std::vector<Lattice> &lattices_out)
{
  // Transform the lattice in wall frame
  int nbr_lattices = lattices_in.size();
  transform_lattices(lattices_in, wall_robot_tf_);

  // Create a lattice for the initial state (in wall frame)
  Lattice initial_lattice;
  add_node(init_state, wall_ocean_tf_, initial_lattice);

  // Handle lattices one by one
  for (int k = 0; k < nbr_lattices; k++)
  {
    Lattice* next_lattice = &(lattices_in[k]);  // lattice to check reachability
    Lattice* prev_lattice;                      // lattice from which propagate motion
    if (k == 0)
      prev_lattice = &initial_lattice;
    else
      prev_lattice = &(lattices_out[k-1]);

    vector<bool> reached(next_lattice->nodes.size(), false);  // whether each node of lattice k has been reached

    // Propagate motion from nodes of the previous lattice
    float wall_yaw;
    Eigen::Vector3f wall_orientation = get_wall_orientation(wall_yaw);  // wall orientation in ocean frame
    vector<int> lat_mult  = {-1, -1, 1, 1};  // multipliers for the maximum rudder commands
    vector<int> elev_mult = {-1, 1, -1, 1};
    float prop_speed = robot_model_.steady_propeller_speed(plan_speed_);   // propeller speed command

    for (int l = 0; l < prev_lattice->nodes.size(); l++ )
    {
      // Transform viewpoint in ocean frame
      geometry_msgs::Pose pose_ocean;
      tf2::doTransform(prev_lattice->nodes[l]->pose, pose_ocean, ocean_wall_tf_);

      // Parse node state
      RobotModel::state_type init_state = to_state(pose_ocean, state_.size());
      init_state[6] = prev_lattice->nodes[l]->speed;

      // Go through extreme control inputs
      vector<geometry_msgs::Pose> propag_poses(4);  // propagated poses (in wall frame)
      float propag_speed = 1000;                    // min of the propagated longitudinal speeds

      for (int m = 0; m < 4; m++) {
        RobotModel::state_type state = init_state;
        Eigen::Vector3f origin; origin << state[0], state[1], state[2];  // origin of motion
        Eigen::Vector3f current_pos;

        float elev_angle = max_elev_rudder_ * elev_mult[m];  // elevation rudder angle command
        float lat_angle  = max_lat_rudder_ * lat_mult[m];    // lateral rudder angle command
        RobotModel::input_type input = {prop_speed, lat_angle, elev_angle, 0};
        float duration = (plan_horizon_ / plan_speed_) / 5;  // duration of each integration

        // Integrate ODE
        float distance = 0.0;      // moved distance in wall direction
        float max_distance = 0.0;  // max moved distance
        Vector3f best_pos;         // position corresponding to the max moved distance
        int safe_cnt = 0;  // counter to go out of the integration loop (the robot could u-turn and never reach the next lattice)

        do {
          robot_model_.integrate(state, input, 0.0, duration, duration/5);
          current_pos << state[0], state[1], state[2];
          distance = wall_orientation.dot(current_pos - origin);

          if (distance > max_distance ) {
            best_pos = current_pos;
            max_distance = distance;
          }
        } while (distance < plan_horizon_ && safe_cnt++ < 20);

        // Transform propagated pose in wall frame
        pose_ocean.position.x = best_pos(0);
        pose_ocean.position.y = best_pos(1);
        pose_ocean.position.z = best_pos(2);
        to_quaternion(0, 0, wall_yaw, pose_ocean.orientation);  // viewpoints parallel to the wall

        tf2::doTransform(pose_ocean, propag_poses[m], wall_ocean_tf_);

        if (state[6] < propag_speed)
          propag_speed = state[6];
      }

      // Get extreme achievable coordinates
      vector<float> x_values(4), z_values(4);  // coordinates of the 4 propagated points
      for (int m = 0; m < 4; m++) {
        x_values[m] = propag_poses[m].position.x;
        z_values[m] = propag_poses[m].position.z;
      }

      float min_x = *(std::min_element(x_values.begin(), x_values.end()));
      float max_x = *(std::max_element(x_values.begin(), x_values.end()));
      float min_z = *(std::min_element(z_values.begin(), z_values.end()));
      float max_z = *(std::max_element(z_values.begin(), z_values.end()));

      // Check viewpoints that can be reached in the next lattice
      for (int m = 0; m < next_lattice->nodes.size(); m++) {
        geometry_msgs::Point node_pos = next_lattice->nodes[m]->pose.position;

        if (node_pos.x >= min_x && node_pos.x <= max_x && node_pos.z >= min_z && node_pos.z <= max_z) {
          prev_lattice->nodes[l]->next.emplace_back(next_lattice->nodes[m]);
          reached[m] = true;
          next_lattice->nodes[m]->speed = propag_speed;
        }
      }
    }

    // Add reachable nodes in the lattice
    for (int l = 0; l < next_lattice->nodes.size(); l++) {
      if (reached[l]) {
        lattices_out[k].nodes.emplace_back(lattices_in[k].nodes[l]);
      }
    }
  }

  // Transform the lattice back in robot frame
  transform_lattices(lattices_out, robot_wall_tf_);
}


bool PlanningNodelet::compute_lattice_gp(Lattice &lattice, ros::Time stamp)
{
  // Compute the corresponding camera orientation for each viewpoint
  int size_lattice = lattice.nodes.size();
  vector<geometry_msgs::Pose> poses(size_lattice);  // camera orientations

  for (int k = 0; k < size_lattice; k++) {
    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::convert(lattice.nodes[k]->pose.orientation, q_orig);
    q_rot.setRPY(M_PI_2, 0.0, 0.0);

    q_new = q_orig * q_rot;
    q_new.normalize();
    tf2::convert(q_new, poses[k].orientation);

    poses[k].position = lattice.nodes[k]->pose.position;
  }

  // Transform the orientation in camera frame
  geometry_msgs::TransformStamped camera_robot_tf;

  try {
    camera_robot_tf = tf_buffer_.lookupTransform(camera_frame_, robot_frame_, stamp);
  }
  catch (tf2::TransformException &ex) {
    NODELET_WARN("[planning_nodelet] %s", ex.what());
    return false;
  }

  for (int k = 0; k < size_lattice; k++) {
    geometry_msgs::Pose transf_pose;
    tf2::doTransform(poses[k], transf_pose, camera_robot_tf);

    poses[k] = transf_pose;
  }

  // Get camera measurement for each viewpoint of the lattice
  mf_sensors_simulator::MultiPoses camera_srv;
  camera_srv.request.stamp = stamp;
  camera_srv.request.pose_array.header.frame_id = robot_frame_;
  camera_srv.request.pose_array.poses = poses;
  camera_srv.request.n_pxl_height = camera_height_;
  camera_srv.request.n_pxl_width = camera_width_;

  if (ray_multi_client_.call(camera_srv)) {
    if (!camera_srv.response.is_success) {
      NODELET_WARN("[planning_nodelet] Call to raycast_multi service didn't give output ");
      return false;
    }
  } else {
    NODELET_WARN("[planning_nodelet] Failed to call raycast_multi service");
    return false;
  }

  // Store camera hit points
  for (int k = 0; k < size_lattice; k++) {
    lattice.nodes[k]->camera_pts_x = camera_srv.response.results[k].x;
    lattice.nodes[k]->camera_pts_y = camera_srv.response.results[k].y;
    lattice.nodes[k]->camera_pts_z = camera_srv.response.results[k].z;
  }

  // Update the GP covariance for each viewpoint
  mf_mapping::UpdateGP gp_srv;
  gp_srv.request.stamp = stamp;
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
      lattice.nodes[k]->gp_cov = gp_srv.response.new_cov_diag[k].data;
    }
  } else {
    NODELET_WARN("[planning_nodelet] Failed to call update_gp service");
    return false;
  }

  return true;
}


void PlanningNodelet::compute_info_gain(Lattice &lattice)
{
  int size_gp = last_gp_mean_.size();

  for (int k = 0; k < lattice.nodes.size(); k++) {
    lattice.nodes[k]->info_gain = 0.0;

    for (int l = 0; l < size_gp; l++) {
      float cov_diff = last_gp_cov_[l][l] - lattice.nodes[k]->gp_cov[l];

      float weight = 0.0;
      if (last_gp_mean_[l] > gp_threshold_)
        weight = 100;

      lattice.nodes[k]->info_gain += weight * cov_diff;
    }
  }
}


std::vector<LatticeNode*> PlanningNodelet::select_viewpoints(const std::vector<Lattice> &lattices)
{
  float best_info_gain = 0.0;
  vector<LatticeNode*> selected_vp;

  for (int k = 0; k < lattices[0].nodes.size(); k++) {
    float info_gain;
    vector<LatticeNode*> vp;

    select_viewpoints(lattices[0].nodes[k].get(), info_gain, vp);

    if (info_gain > best_info_gain) {
      selected_vp = vp;
      best_info_gain = info_gain;
    }
  }

  return selected_vp;
}


void PlanningNodelet::select_viewpoints(
  LatticeNode* node,
  float &info_gain,
  std::vector<LatticeNode*> &selected_vp)
{
  // Terminating condition
  if (node->next.size() == 0) {
    info_gain = node->info_gain;
    selected_vp.resize(1);
    selected_vp[0] = node;

    return;
  }

  // Select best next node
  float best_info_gain = 0.0;

  for (int k = 0; k < node->next.size(); k++) {
    float info_gain;
    vector<LatticeNode*> vp;

    select_viewpoints(node->next[k].get(), info_gain, vp);

    if (info_gain > best_info_gain) {
      selected_vp = vp;
      best_info_gain = info_gain;
    }
  }

  // Prepare output
  info_gain = best_info_gain + node->info_gain;
  selected_vp.insert(selected_vp.begin(), node);
}


void PlanningNodelet::fill_display_obj(
  const std::vector<Lattice> &lattices,
  const std::vector<LatticeNode*> &selected_vp)
{
  // Fill selected viewpoints poses and camera hitpoints
  selected_vp_.resize(nbr_lattices_);
  x_hit_pt_sel_.resize(0);
  y_hit_pt_sel_.resize(0);
  z_hit_pt_sel_.resize(0);

  for (int k = 0; k < nbr_lattices_; k++) {
    // Fill selected pose
    geometry_msgs::Pose pose;
    tf2::doTransform(selected_vp[k]->pose, pose, ocean_robot_tf_);
    selected_vp_[k] = pose;

    // Fill camera hit points
    x_hit_pt_sel_.insert(x_hit_pt_sel_.end(), selected_vp[k]->camera_pts_x.begin(), selected_vp[k]->camera_pts_x.end());
    y_hit_pt_sel_.insert(y_hit_pt_sel_.end(), selected_vp[k]->camera_pts_y.begin(), selected_vp[k]->camera_pts_y.end());
    z_hit_pt_sel_.insert(z_hit_pt_sel_.end(), selected_vp[k]->camera_pts_z.begin(), selected_vp[k]->camera_pts_z.end());
  }

  for (int k = 0; k < x_hit_pt_sel_.size(); k++) {
    // Transform the hit points in ocean frame
    geometry_msgs::Point pose, tmp;
    pose.x = x_hit_pt_sel_[k];
    pose.y = y_hit_pt_sel_[k];
    pose.z = z_hit_pt_sel_[k];

    tf2::doTransform(pose, tmp, ocean_camera_tf_);

    x_hit_pt_sel_[k] = tmp.x;
    y_hit_pt_sel_[k] = tmp.y;
    z_hit_pt_sel_[k] = tmp.z;
  }

  // Build lattice of non-selected viewpoints (in ocean frame)
  lattice_.resize(0);

  for (int k = 0; k < nbr_lattices_; k++) {
    for (int l = 0; l < lattices[k].nodes.size(); l++) {
      if (lattices[k].nodes[l].get() != selected_vp[k])  {
        geometry_msgs::Pose pose;
        tf2::doTransform(lattices[k].nodes[l]->pose, pose, ocean_robot_tf_);
        lattice_.emplace_back(pose);
      }
    }
  }
}


void PlanningNodelet::generate_path(
  const std::vector<geometry_msgs::Pose> &selected_vp,
  nav_msgs::Path &path,
  std::vector<geometry_msgs::Pose> &waypoints)
{
  path.poses.resize(0);
  waypoints.resize(1);

  if (waypoints_.size() == 0 | replan_)
    waypoints[0] = tf_to_pose(ocean_robot_tf_);
  else
    waypoints[0] = waypoints_.back();

  waypoints.insert(waypoints.end(), selected_vp_.begin(), selected_vp_.end());

  if (linear_path_) {
    for (int k = 0; k < nbr_lattices_; k++) {
      nav_msgs::Path intermed_path = straight_line_path(waypoints[k], waypoints[k+1], path_res_);

      if (k == 0)
        path.poses = intermed_path.poses;
      else
        path.poses.insert(path.poses.end(), intermed_path.poses.begin()+1, intermed_path.poses.end());
    }
  } else {
    path = spline_path(waypoints, path_res_, plan_speed_);
  }
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


nav_msgs::Path PlanningNodelet::spline_path(
  const std::vector<geometry_msgs::Pose> &waypoints,
  float resolution,
  float speed)
{
  int n = waypoints.size();
  vector<Eigen::Vector3f> p(n);  // waypoints positions
  vector<Eigen::Vector3f> o(n);  // waypoints orientations

  for (int k = 0; k < n; k++) {
    p[k] << waypoints[k].position.x, waypoints[k].position.y, waypoints[k].position.z;

    double roll, pitch, yaw;
    to_euler(waypoints[k].orientation, roll, pitch, yaw);
    o[k] << cos(yaw), sin(yaw), sin(pitch);
  }

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
  vector<Lattice> lattices_tmp(nbr_lattices_);

  if (mult_lattices_)
  {
    RobotModel::state_type state;

    if (waypoints_.size() == 0 || replan_) {
      state = state_;
    } else {
      state = to_state(waypoints_.back(), state_.size());
    }

    generate_lattices(state, lattices_tmp);
  }
  else if (cart_lattice_)
  {
    float lat_turn_radius  = robot_model_.lat_turn_radius(plan_speed_, max_lat_rudder_);
    float elev_turn_radius = robot_model_.elev_turn_radius(plan_speed_, max_lat_rudder_);

    float max_lat_angle  = plan_horizon_ / (2 * lat_turn_radius);
    float max_elev_angle = plan_horizon_ / (2 * elev_turn_radius);

    if (!horiz_motion_) max_lat_angle = 0;
    if (!vert_motion_)  max_elev_angle = 0;

    generate_cart_lattice(max_lat_angle, max_elev_angle, plan_horizon_, lattice_res_, lattices_tmp[0]);
  }
  else
  {
    generate_lattice(lattices_tmp[0]);
  }

  // Filter the lattices (remove viewpoints that are not geometrically acceptable)
  vector<Lattice> lattices_tmp2(nbr_lattices_);

  for (int k = 0; k < nbr_lattices_; k++) {
    filter_lattice(lattices_tmp[k], lattices_tmp2[k]);
  }

  // Connect lattices and remove viewpoints that are not dynamically reachable
  vector<Lattice> lattices(nbr_lattices_);
  connect_lattices(state_, lattices_tmp2, lattices);

  for (int k = 0; k < nbr_lattices_; k++) {
    if (lattices[k].nodes.size() == 0) {
      NODELET_WARN("[planning_nodelet] Lattice %d is empty", k);
      return false;
    }
  }

  // Update the GP covariance for each viewpoint
  ros::Time stamp = ocean_robot_tf_.header.stamp;  // time at which to fetch the ROS transforms

  for (int k = 0; k < nbr_lattices_; k++) {
    bool ret = compute_lattice_gp(lattices[k], stamp);

    if (!ret) {
      NODELET_WARN("[planning_nodelet] Error when computing GP covariance ");
      return false;
    }
  }

  // Compute information gains and select best viewpoints
  for (int k = 0; k < nbr_lattices_; k++) {
    compute_info_gain(lattices[k]);
  }

  vector<LatticeNode*> selected_vp = select_viewpoints(lattices);

  if (selected_vp.size() != nbr_lattices_) {
    NODELET_WARN("[planning_nodelet] Not the right number of selected viewpoints (%d against %d)",
      (int)selected_vp.size(), (int)nbr_lattices_
    );
    return false;
  }

  // Fill objects for display purposes
  fill_display_obj(lattices, selected_vp);

  // Generate a spline trajectory (in ocean frame) between the selected points
  nav_msgs::Path new_path;
  vector<geometry_msgs::Pose> waypoints;  // new viewpoints in ocean frame
  generate_path(selected_vp_, new_path, waypoints);

  if (new_path.poses.size() <= 1) {
    NODELET_WARN("[planning_nodelet] Computed path to small (size: %d)", (int)new_path.poses.size());
    return false;
  }

  // Append the path to the previous path
  path_.header.stamp = ros::Time::now();

  if (replan_) {
    path_.poses = new_path.poses;
    waypoints_ = waypoints;
  }
  else {
    if (path_.poses.size() == 0) {
      path_.poses.emplace_back(new_path.poses[0]);
      waypoints_.emplace_back(waypoints[0]);
    }

    path_.poses.insert(path_.poses.end(), new_path.poses.begin()+1, new_path.poses.end());
    waypoints_.insert(waypoints_.end(), waypoints.begin()+1, waypoints.end());
  }

  return true;
}


}  // namespace mfcpp
