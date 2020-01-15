/**
 * @file
 *
 * \brief  Definition of a Model Predictive Controler for an underwater robot
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#include "mpc_nodelet.hpp"
#include "mf_common/common.hpp"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <vector>

using namespace std;


namespace mfcpp {

std::vector<geometry_msgs::Pose> MPCNodelet::adapt_path(
  const std::vector<geometry_msgs::Pose> &orig_path,
  const geometry_msgs::Point &current_position,
  int nbr_steps,
  float &spatial_resolution,
  float &time_resolution)
{
  // Find the current position on the path
  int size_path = orig_path.size();
  float min_dist = 10000;
  int idx_path = -1;

  for (int k = 0; k < size_path; k++) {
    float d = distance2(current_position, orig_path[k].position);

    if (d < min_dist) {
      min_dist = d;
      idx_path = k;
    }
  }

  // Handle case when the robot is already at the end of the path
  if (idx_path >= size_path - 1) {
    return vector<geometry_msgs::Pose>(nbr_steps, orig_path[size_path-1]);
  }

  // Update resolutions if path is too short
  float path_length = 0;

  for (int k = idx_path; k < size_path-1; k++) {
    path_length += distance(orig_path[k], orig_path[k+1]);
  }

  if (spatial_resolution*nbr_steps > path_length) {
    float tmp = spatial_resolution;
    spatial_resolution = path_length / nbr_steps;
    time_resolution *= spatial_resolution / tmp;

  }

  // Interpolate the path to have the right spatial resolution
  vector<geometry_msgs::Pose> new_path(nbr_steps + 1);
  new_path[0] = orig_path[idx_path];

  for (int k = 1; k <= nbr_steps; k++) {
    float s = 0;   // curvilinear abscissa
    float ds = distance(new_path[k-1], orig_path[idx_path + 1]);  // distance to the next waypoint in the path

    while (s + ds < spatial_resolution && idx_path+2 < size_path) {
      idx_path++;
      s += ds;
      ds = distance(orig_path[idx_path], orig_path[idx_path+1]);
    }

    if (s + ds < spatial_resolution) {
      // Last pose of the path
      if (k != nbr_steps) {
        // TODO: to remove
        NODELET_WARN("[mpc_nodelet] Unexpected path length");
      }

      new_path[k] = orig_path[size_path-1];
    } else {
      float t = (spatial_resolution-s) / ds;
      new_path[k] = interpolate(orig_path[idx_path], orig_path[idx_path+1], t);
    }
  }

  return new_path;
}


void MPCNodelet::compute_control(
  const nav_msgs::Path &path,
  const vector<float> &current_state,
  float nominal_speed,
  float time_horizon,
  int nbr_steps)
{
  if (path.poses.size() < 2) {
    NODELET_WARN("[mpc_nodelet] Path size < 2");
    return;
  }

  // Parse current state
  geometry_msgs::Point current_position;
  current_position.x = current_state[0];
  current_position.y = current_state[1];
  current_position.z = current_state[2];

  // Store the original path for convenience
  int size_path = path.poses.size();
  vector<geometry_msgs::Pose> orig_path(size_path);

  for (int k = 0; k < size_path; k++) {
    orig_path[k] = path.poses[k].pose;
  }

  // Get path with suitable length and resolution
  float spatial_resolution = nominal_speed * time_horizon / nbr_steps;  // spatial resolution
  float time_resolution = time_horizon / nbr_steps;
  vector<geometry_msgs::Pose> new_path = adapt_path(orig_path, current_position,
    nbr_steps, spatial_resolution, time_resolution);

  // Fill reference points for MPC optimisation
  vector<vector<float>> x_ref(nbr_steps+1, vector<float>(current_state.size(), 0));
  // TODO: change it for Eigen vectors?
  // ....


}

}  // namespace mfcpp
