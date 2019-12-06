/**
 * @file
 *
 * \brief  Definition of path planning functions
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "planning_nodelet.hpp"
#include "mf_sensors_simulator/MultiPoses.h"
#include "mf_mapping/UpdateGP.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <vector>
#include <iostream>

using namespace std;
using Eigen::Vector3d;


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


void PlanningNodelet::generate_lattice(float max_lat_angle, float max_elev_angle,
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

          // Transform point in wall frame to check for collision
          geometry_msgs::Pose transf_pose;
          tf2::doTransform(pose, transf_pose, wall_robot_tf_);

          if (transf_pose.position.z >= min_wall_dist_) {
            // Account for rotation needed to reach the waypoint
            tf2::Quaternion q_orig, q_rot, q_new;
            q_orig.setRPY(0, 0, 0);
            q_rot.setRPY(0.0, -elev_angle, lat_angle);

            q_new = q_rot * q_orig;
            q_new.normalize();
            tf2::convert(q_new, pose.orientation);

            // Add the waypoint to the lattice
            lattice.emplace_back(pose);
          }
        }

        z += resolution;
      }

      y += resolution;
    }

    x += resolution;
  }
}


bool PlanningNodelet::plan_trajectory()
{
  // Check for Gaussian Process received
  int size_gp = last_gp_cov_.size();

  if (last_gp_cov_.size() == 0 || last_gp_mean_.size() == 0)
    return false;

  // Compute maximum angles for lattice generation
  float lat_turn_radius = robot_model_.lat_turn_radius(plan_speed_, max_lat_rudder_);
  float elev_turn_radius = robot_model_.elev_turn_radius(plan_speed_, max_lat_rudder_);

  float max_lat_angle = plan_horizon_ / (2 * lat_turn_radius);
  float max_elev_angle = plan_horizon_ / (2 * elev_turn_radius);

  if (!horiz_motion_) max_lat_angle = 0;
  if (!vert_motion_)  max_elev_angle = 0;

  // Generate a lattice of possible waypoints (in robot frame)
  generate_lattice(max_lat_angle, max_elev_angle, plan_horizon_, lattice_res_, lattice_);

  // Compute the corresponding camera orientation for each viewpoint
  int size_lattice = lattice_.size();

  for (int k = 0; k < size_lattice; k++) {
    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::convert(lattice_[k].orientation, q_orig);
    q_rot.setRPY(M_PI_2, 0.0, 0.0);

    q_new = q_orig * q_rot;
    q_new.normalize();
    tf2::convert(q_new, lattice_[k].orientation);
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
    tf2::doTransform(lattice_[k], transf_pose, camera_robot_tf);

    lattice_[k] = transf_pose;
  }

  // Get camera measurement for each viewpoint of the lattice
  mf_sensors_simulator::MultiPoses camera_srv;
  camera_srv.request.pose_array.header.frame_id = robot_frame_;
  camera_srv.request.pose_array.poses = lattice_;
  camera_srv.request.n_pxl_height = camera_height_;
  camera_srv.request.n_pxl_width = camera_width_;

  if (ray_multi_client_.call(camera_srv)) {
    if (!camera_srv.response.is_success) {
      return false;
    }
  } else {
    NODELET_WARN("[planning_nodelet] Failed to call raycast_multi service");
    return false;
  }

  // Update the GP covariance for each viewpoint
  vector<vector<float>> cov_diag(size_lattice);  // diagonal of the updated GP for each view point

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

  // Compute information gain and select best viewpoint
  vector<float> info_gain(size_lattice, 0.0);

  for (int k = 0; k < size_lattice; k++) {
    for (int l = 0; l < size_gp; l++) {
      float cov_diff = last_gp_cov_[l][l] - cov_diag[k][l];
      float weight = 100 * (1/(1 + exp(-gp_weight_*(last_gp_mean_[l] - 0.5))));
      info_gain[k] += weight * cov_diff;
    }
  }

  selected_vp_ = std::max_element(info_gain.begin(), info_gain.end()) - info_gain.begin();

  x_hit_pt_sel_ = camera_srv.response.results[selected_vp_].x;
  y_hit_pt_sel_ = camera_srv.response.results[selected_vp_].y;
  z_hit_pt_sel_ = camera_srv.response.results[selected_vp_].z;

}


}  // namespace mfcpp
