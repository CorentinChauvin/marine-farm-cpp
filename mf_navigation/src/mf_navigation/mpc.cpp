/**
 * @file
 *
 * \brief  Definition of a Model Predictive Controler for an underwater robot
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#include "mpc_nodelet.hpp"
#include "mf_common/common.hpp"
#include "osqp.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <vector>

using namespace std;
using Eigen::VectorXf;
using Eigen::VectorXd;
using Eigen::MatrixXf;
using Eigen::MatrixXd;


namespace mfcpp {

std::vector<geometry_msgs::Pose> MPCNodelet::adapt_path(
  const std::vector<geometry_msgs::Pose> &orig_path,
  const geometry_msgs::Point &current_position,
  int nbr_steps,
  float &spatial_resolution,
  float &time_resolution,
  float &desired_speed)
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
    float factor = path_length / (nbr_steps * spatial_resolution);
    spatial_resolution *= factor;
    time_resolution *= factor;
    desired_speed *= factor;
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


template <class VectorT>
void MPCNodelet::fill_ref_pts(
  int N, int n, int m,
  const std::vector<geometry_msgs::Pose> &path,
  float desired_speed,
  float last_desired_speed,
  const RobotModel &robot_model,
  VectorT &X_ref,
  VectorT &U_ref)
{
  if (N < 1 || n < 1 || m < 1 || path.size() != N + 1) {
    NODELET_WARN("[mpc_nodelet] Wrong input dimensions in fill_ref_pts");
    return;
  }

  X_ref = VectorT::Zero(n * (N+1));
  U_ref = VectorT::Zero(m * (N+1));

  for (int k = 0; k <= N; k++) {
    double roll, pitch, yaw;
    to_euler(path[k].orientation, roll, pitch, yaw);

    X_ref(k*n)     = path[k].position.x;
    X_ref(k*n + 1) = path[k].position.y;
    X_ref(k*n + 2) = path[k].position.z;
    X_ref(k*n + 3) = roll;
    X_ref(k*n + 4) = pitch;
    X_ref(k*n + 5) = yaw;
    X_ref(k*n + 6) = desired_speed;
  }

  U_ref(0) = robot_model.steady_propeller_speed(last_desired_speed);
  float desired_prop_speed = robot_model.steady_propeller_speed(desired_speed);

  for (int k = 1; k <= N; k++) {
    U_ref(k*m) = desired_prop_speed;
  }
}


template <class MatrixT>
void MPCNodelet::fill_G(const MatrixT &Ad, const MatrixT &Bd, int N, MatrixT &G)
{
  int n = Bd.rows();
  int m = Bd.cols();
  G = MatrixT::Zero(n*(N+1), m*(N+1));
  MatrixT M = Bd;  // Ad^k * Bd

  for (int k = 1; k <= N; k++) {
    if (k != 1)
      M = Ad * M;

    for (int l = 0; l < N-k+1; l++) {
      G.block((k+l)*n, (1+l)*m, n, m) = M;
    }
  }
}


template <class MatrixT>
void MPCNodelet::fill_H(const MatrixT &Ad, int N, MatrixT &H)
{
  int n = Ad.rows();
  H = MatrixT(n*(N+1), n);

  H.block(0, 0, n, n).setIdentity();
  MatrixT M = MatrixT::Identity(n, n);

  for (int k = 1; k <= N; k++) {
    M = M * Ad;
    H.block(k*n, 0, n, n) = M;
  }
}


template <class MatrixT>
void MPCNodelet::fill_L(const MatrixT &P, const MatrixT &Q_x, int N, MatrixT &L)
{
  int n = P.rows();
  L = MatrixT::Zero(n*(N+1), n*(N+1));

  for (int k = 0; k < N; k++) {
    L.block(k*n, k*n, n, n) = Q_x;
  }
  L.block(N*n, N*n, n, n) = P;
}


template <class MatrixT>
void MPCNodelet::fill_M(const MatrixT &R_u, const MatrixT &R_delta, int N, MatrixT &M)
{
  int m = R_u.rows();
  M = MatrixT::Zero(m*(N+1), m*(N+1));

  // Diagonal blocks
  MatrixT A = 2*R_delta + R_u;

  for (int k = 0; k <= N; k++) {
    if (k == 0 || k == N)
      M.block(k*m, k*m, m, m) = R_delta;
    else
      M.block(k*m, k*m, m, m) = A;
  }

  // Over-diagonal blocks
  MatrixT B = -2*R_delta;

  for (int k = 0; k < N; k++) {
    M.block(k*m, (k+1)*m, m, m) = B;
  }
}


template <class VectorT, class MatrixT>
void MPCNodelet::fill_V(float desired_speed, float last_desired_speed,
  const MatrixT &R_delta, int N, VectorT &V)
{
  int m = R_delta.rows();
  V = VectorT::Zero(m*(N+1));
  VectorT u_0_ref(m), u_m1_ref(m);  // current and last reference control signal

  u_0_ref  << desired_speed, 0, 0, 0;
  u_m1_ref << last_desired_speed, 0, 0, 0;
  VectorT v = 2 * R_delta * (u_0_ref - u_m1_ref);

  V.block(0, 0, m, 1) = -v;
  V.block(m, 0, m, 1) = v;
}


template <class VectorT, class MatrixT>
void MPCNodelet::fill_bounds_objs(
  const MPCBounds &bounds,
  int n, int N,
  const VectorT &X0, const VectorT &X_ref,
  const MatrixT &G, const MatrixT &H,
  VectorT &lb, VectorT &ub, MatrixT &Ab)
{
  // Bounds on the state
  VectorT a = VectorT::Constant(N+1, -bounds.delta_m);
  VectorT b = VectorT::Constant(N+1, bounds.delta_m);
  MatrixT Kb = MatrixT::Zero(N+1, n*(N+1));

  for (int k = 0; k <= N; k++) {
    Kb(k, n-1 + k*n) = 1;
  }

  // Bounds on the input
  int m = bounds.input.size();
  VectorT c(m), d(m);

  for (int k = 0; k < m; k++) {
    c(k) = -bounds.input[k];
    d(k) = bounds.input[k];
  }
  c(0) = 0;

  // Total bounds
  lb = VectorT::Zero((m+1)*(N+1));
  ub = VectorT::Zero((m+1)*(N+1));
  Ab = MatrixT::Zero((m+1)*(N+1), m*(N+1));

  lb.block(0, 0, N+1, 1)       = a - Kb*H*X0 - Kb*X_ref;
  ub.block(0, 0, N+1, 1)       = b - Kb*H*X0 - Kb*X_ref;

  for (int k = 0; k < N+1; k++) {
    lb.block(N+1 + k*m, 0, m, 1) = c;
    ub.block(N+1 + k*m, 0, m, 1) = d;
  }

  Ab.block(0, 0, N+1, m*(N+1)) = Kb*G;
  Ab.block(N+1, 0, m*(N+1), m*(N+1)) = MatrixT::Identity(m, m);
}


void MPCNodelet::compute_control(
  const nav_msgs::Path &path,
  const vector<float> &current_state,
  const vector<float> &last_control,
  const RobotModel &robot_model,
  const MPCTuningParameters &tuning_params,
  float &desired_speed,
  float last_desired_speed,
  float time_horizon,
  int nbr_steps,
  const MPCBounds &bounds)
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

  int n = current_state.size();
  int m = last_control.size();
  VectorXf X0(n);
  VectorXf U0(m);

  for (int k = 0; k < n; k++) X0(k) = current_state[k];
  for (int k = 0; k < m; k++) U0(k) = last_control[k];

  // Store the original path for convenience
  int size_path = path.poses.size();
  vector<geometry_msgs::Pose> orig_path(size_path);

  for (int k = 0; k < size_path; k++) {
    orig_path[k] = path.poses[k].pose;
  }

  // Get path with suitable length and resolution
  float spatial_resolution = desired_speed * time_horizon / nbr_steps;  // spatial resolution
  float time_resolution = time_horizon / nbr_steps;

  vector<geometry_msgs::Pose> new_path = adapt_path(orig_path, current_position,
    nbr_steps, spatial_resolution, time_resolution, desired_speed);

  // Fill reference points for MPC optimisation
  int N = nbr_steps;
  VectorXf X_ref, U_ref;

  fill_ref_pts(N, n, m, new_path, desired_speed, last_desired_speed, robot_model,
     X_ref, U_ref);

  // Build MPC objects
  MatrixXf Ad, Bd;  // discretised model
  MatrixXf G, H;    // to express X with respect to U and X0
  MatrixXf L, M;    // to compute quadratic partts of the cost function
  VectorXf V;       // to compute linear parts of the cost function

  robot_model.get_lin_discr_matrices(X0, U0, Ad, Bd, time_resolution, 10);
  fill_G(Ad, Bd, N, G);
  fill_H(Ad, N, H);
  fill_L(tuning_params_.P, tuning_params_.Q_x, N, L);
  fill_M(tuning_params_.R_u, tuning_params_.R_delta, N, M);
  fill_V(desired_speed, last_desired_speed, tuning_params_.R_delta, N, V);

  // Build MPC bounds objects
  VectorXf lb, ub;  // lower and upper bounds
  MatrixXf Ab;  // multiplicative factor in front of U in the bound inequalities

  fill_bounds_objs(bounds, n, N, X0, X_ref, G, H, lb, ub, Ab);

}

}  // namespace mfcpp
