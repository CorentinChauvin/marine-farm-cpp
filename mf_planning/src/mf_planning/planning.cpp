/**
 * @file
 *
 * \brief  Definition of path planning functions
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "planning_nodelet.hpp"
#include "mf_sensors_simulator/MultiPoses.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <vector>
#include <iostream>

using namespace std;
using Eigen::Vector3d;


namespace mfcpp {

void PlanningNodelet::generate_lattice(float max_lat_angle, float max_elev_angle)
{
  int n_x = plan_horizon_ / lattice_res_ + 1;  // size of the lattice in x direction
  int n_y = plan_horizon_ * sin(max_lat_angle) / lattice_res_;   // half size in y direction
  int n_z = plan_horizon_ * sin(max_elev_angle) / lattice_res_;  // half size in z direction
  unsigned int n_lattice = n_x * (2*n_y + 1) * (2*n_z + 1);      // total size of the lattice

  lattice_.resize(0);
  lattice_.reserve(n_lattice);
  float x = 0;

  for (int i = 0; i < n_x; i++) {
    float y = -n_y * lattice_res_;

    for (int j = -n_y; j <= n_y; j++) {
      float z = -n_z * lattice_res_;

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
          geometry_msgs::Pose transf_pose = pose;
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
            lattice_.emplace_back(pose);
          }
        }

        z += lattice_res_;
      }

      y += lattice_res_;
    }

    x += lattice_res_;
  }
}


void PlanningNodelet::plan_trajectory()
{
  // Compute maximum angles for lattice generation
  float lat_turn_radius = robot_model_.lat_turn_radius(plan_speed_, max_lat_rudder_);
  float elev_turn_radius = robot_model_.elev_turn_radius(plan_speed_, max_lat_rudder_);

  float max_lat_angle = plan_horizon_ / (2 * lat_turn_radius);
  float max_elev_angle = plan_horizon_ / (2 * elev_turn_radius);

  // Generate a lattice of possible waypoints (in robot frame)
  generate_lattice(max_lat_angle, max_elev_angle);

  // Raycast
  mf_sensors_simulator::MultiPoses srv;
  srv.request.pose_array.header.frame_id = robot_frame_;
  srv.request.pose_array.poses = lattice_;
  srv.request.n_pxl_height = camera_height_;
  srv.request.n_pxl_width = camera_width_;

  if (ray_multi_client_.call(srv)) {
    if (srv.response.is_success) {
      // TODO: do some stuff
      // ...
    }
  } else {
    NODELET_WARN("[planning_nodelet] Failed to call raycast_multi service.");
  }

}


}  // namespace mfcpp
