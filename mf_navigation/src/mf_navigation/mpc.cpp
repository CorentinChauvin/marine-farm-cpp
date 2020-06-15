/**
 * @file
 *
 * \brief  Definition of a Model Predictive Controler for an underwater robot
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#include "mpc_node.hpp"
#include "mf_common/common.hpp"
#include "osqp.h"
#include "osqp_eigen/SparseMatrixHelper.hpp"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <vector>

using namespace std;
using Eigen::VectorXf;
using Eigen::VectorXd;
using Eigen::MatrixXf;
using Eigen::MatrixXd;


namespace mfcpp {

std::vector<geometry_msgs::Pose> MPCNode::adapt_path(
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
    return vector<geometry_msgs::Pose>(nbr_steps+1, orig_path[size_path-1]);
  }

  // Compute length of the path
  float path_length = 0;

  for (int k = idx_path; k < size_path-1; k++) {
    path_length += distance(orig_path[k], orig_path[k+1]);
  }

  // Update resolutions if path is too short
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
        ROS_WARN("[mpc_node] Unexpected path length");
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
bool MPCNode::fill_ref_pts(
  int N, int n, int m,
  const std::vector<geometry_msgs::Pose> &path,
  float desired_speed,
  float last_desired_speed,
  const RobotModel &robot_model,
  VectorT &X_ref,
  VectorT &U_ref)
{
  if (N < 1 || n < 1 || m < 1 || path.size() != N+1) {
    ROS_WARN("[mpc_node] Wrong input dimensions in fill_ref_pts:");
    ROS_WARN("[mpc_node] path size: %d ; N+1=%d", (int) path.size(), N+1);
    return false;
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

  for (int k = 0; k <= N; k++) {
    U_ref(k*m) = desired_prop_speed;
  }

  return true;
}


template <class VectorT>
void MPCNode::modulo_ref_state(VectorT &X_ref, const VectorT &x0, int n)
{
  int N = X_ref.rows() / n;

  // For each three angles
  for (int k = 0; k < 3; k++) {
    // Handle initial error
    while (abs(X_ref(3 + k) - x0(3 + k)) >= M_PI) {
      if (X_ref(3 + k) - x0(3 + k) >= M_PI)
        X_ref(3 + k) -= 2*M_PI;
      else
        X_ref(3 + k) += 2*M_PI;
    }

    // Next reference points
    for (int l = 1; l < N; l++) {
      while (abs(X_ref(l*n + 3 + k) - X_ref((l-1)*n + 3 + k)) >= M_PI) {
        if (X_ref(l*n + 3 + k) - X_ref((l-1)*n + 3 + k) >= M_PI)
          X_ref(l*n + 3 + k) -= 2*M_PI;
        else
          X_ref(l*n + 3 + k) += 2*M_PI;
      }
    }
  }
}


template <class VectorT, class MatrixT>
void MPCNode::fill_ltv_G_H_D(const VectorT &X_ref, const VectorT &U_ref,
  int N, float dt, float ds,
  MatrixT &G, MatrixT &H, VectorT &D)
{
  int n = X_ref.rows() / (N+1);
  int m = U_ref.rows() / (N+1);
  G = MatrixT::Zero(n*N, m*N);
  H = MatrixT::Zero(n*N, n);
  D = VectorT::Zero(n*N);

  // Linearise the system at each reference point
  MatrixT Ad_0;  // Ad[0] matrix (linearised at first reference point)
  vector<MatrixT> Ad_m(N);  // multiples of Ad[k]: Ad_m[k] = Ad[k]*Ad[k-1]*...*Ad[1] (Ad_m[0]=identity)
  vector<MatrixT> Ad(N);  // Ad matrices
  vector<MatrixT> Bd(N);  // Bd matrices

  for (int k = 0; k < N; k++) {
    MatrixT _Ad, _Bd;
    VectorT x_ref = X_ref.block(k*n, 0, n, 1);
    VectorT u_ref = U_ref.block(k*m, 0, m, 1);
    robot_model_.get_lin_discr_matrices(x_ref, u_ref, _Ad, _Bd, dt);

    if (k == 0) {
      Ad_0 = _Ad;
      Ad_m[0] = MatrixT::Identity(n, n);
    } else if (k == 1)
      Ad_m[1] = _Ad;
    else if (k > 1)
      Ad_m[k] = _Ad * Ad_m[k-1];

    Ad[k] = _Ad;
    Bd[k] = _Bd;
  }

  // Build G (column by column)
  for (int j = 0; j < N; j++) {
    G.block(0, j*m, n, m) = Bd[j];
    MatrixT prod = Bd[j];  // product of Ad and Bd: prod = Ad(i)...Ad(j+1)Bd(j)

    for (int i = j+1; i < N; i++) {
      prod = Ad[i] * prod;
      G.block(i*n, j*m, n, m) = prod;
    }
  }

  // Build H
  H.block(0, 0, n, n) = Ad[0];
  MatrixT prod = Ad[0];

  for (int i = 1; i < N; i++) {
    prod = Ad[i] * prod;
    H.block(i*n, 0, n, n) = prod;
  }

  // Build D
  VectorT delta;  // k-th reference error

  for (int k = 0; k < N; k++) {
    // Evaluate the ODE at the k-th reference point
    RobotModel::state_type x_k_ref(n);
    RobotModel::input_type u_k_ref(m);
    RobotModel::state_type _f_k(n);

    for (int l = 0; l < n; l++)
      x_k_ref[l] = X_ref(k*n + l);
    for (int l = 0; l < m; l++)
      u_k_ref[l] = U_ref(k*m + l);

    robot_model_.eval_ode(x_k_ref, u_k_ref, _f_k, 0.0);

    VectorT f_k(n);
    for (int l = 0; l < n; l++)
      f_k(l) = _f_k[l];

    // Compute k-th reference error
    delta = X_ref.block(k*n, 0, n, 1) + dt*f_k - X_ref.block((k+1)*n, 0, n, 1);

    // Fill the k-th row of D
    if (k == 0)
      D.block(k*n, 0, n, 1) = delta;
    else
      D.block(k*n, 0, n, 1) = Ad[k]*D.block((k-1)*n, 0, n, 1) + delta;

    // for (int l = 0; l <= k; l++) {
    //   D.block(k*n, 0, n, 1) += Ad_m[k-l] * delta[l];
    // }
  }
}


template <class MatrixT>
void MPCNode::fill_lti_G(const MatrixT &Ad, const MatrixT &Bd, int N, MatrixT &G)
{
  int n = Bd.rows();
  int m = Bd.cols();
  G = MatrixT::Zero(n*N, m*N);
  MatrixT M = Bd;  // Ad^k * Bd

  for (int k = 0; k < N; k++) {
    if (k != 0)
      M = Ad * M;

    for (int l = 0; l < N-k; l++) {
      G.block((k+l)*n, l*m, n, m) = M;
    }
  }
}


template <class MatrixT>
void MPCNode::fill_lti_H(const MatrixT &Ad, int N, MatrixT &H)
{
  int n = Ad.rows();
  H = MatrixT(n*N, n);
  MatrixT M = MatrixT::Identity(n, n);

  for (int k = 0; k < N; k++) {
    M = M * Ad;
    H.block(k*n, 0, n, n) = M;
  }
}


template <class MatrixT>
void MPCNode::fill_L(const MatrixT &P, const MatrixT &Q_x, int N, MatrixT &L)
{
  int n = P.rows();
  L = MatrixT::Zero(n*N, n*N);

  for (int k = 0; k < N-1; k++) {
    L.block(k*n, k*n, n, n) = Q_x;
  }
  L.block((N-1)*n, (N-1)*n, n, n) = P;
}


template <class MatrixT>
void MPCNode::fill_M(const MatrixT &R_u, const MatrixT &R_delta, int N, MatrixT &M)
{
  int m = R_u.rows();
  M = MatrixT::Zero(m*N, m*N);

  // Diagonal blocks
  MatrixT A = 2*R_delta + R_u;

  for (int k = 0; k < N; k++) {
    if (k < N-1)
      M.block(k*m, k*m, m, m) = A;
    else
      M.block(k*m, k*m, m, m) = R_delta + R_u;
  }

  // Over-diagonal blocks
  MatrixT B = -R_delta;

  for (int k = 0; k < N-1; k++) {
    M.block(k*m, (k+1)*m, m, m) = B;
    M.block((k+1)*m, k*m, m, m) = B;
  }
}


template <class VectorT, class MatrixT>
void MPCNode::fill_V(VectorT control_ref, VectorT last_control,
  const MatrixT &R_delta, int N, VectorT &V)
{
  int m = R_delta.rows();
  V = VectorT::Zero(m*N);
  V.block(0, 0, m, 1) = 2*R_delta*(control_ref-last_control);
}


template <class MatrixT>
void MPCNode::fill_lti_LG(const MatrixT &G, const MatrixT &P, const MatrixT &Q_x,
  int n, int m, int N, MatrixT &LG)
{
  LG = MatrixXf::Zero(n*N, m*N);

  for (int k = 0; k < N-1; k++) {
    MatrixXf M = Q_x * G.block(k*n, 0, n, m);

    for (int l = 0; l < N-k-1; l++) {
      LG.block((k+l)*n, l*m, n, m) = M;
    }
  }
  for (int l = 0; l < N; l++) {
    LG.block((N-1)*n, l*m, n, m) = P * G.block((N-1)*n, l*m, n, m);
  }
}


template <class VectorT, class MatrixT>
void MPCNode::fill_bounds_objs(
  const MPCBounds &bounds,
  int n, int N,
  const VectorT &X0, const VectorT &X_ref, const VectorT &U_ref,
  const MatrixT &G, const MatrixT &H, const VectorT &D,
  VectorT &lb, VectorT &ub, MatrixT &Ab)
{
  // Bounds on the state
  VectorT a = VectorT::Constant(N, -bounds.delta_m);
  VectorT b = VectorT::Constant(N, bounds.delta_m);
  MatrixT Kb = MatrixT::Zero(N, n*N);

  for (int k = 0; k < N; k++) {
    Kb(k, n-1 + k*n) = 1;
  }

  // Bounds on the input
  int m = bounds.input.size();
  VectorT c(m), d(m);

  for (int k = 0; k < m; k++) {
    c(k) = -bounds.input[k];
    d(k) = bounds.input[k];
  }
  c(0) = 0;  // the submarine can't go backwards

  // Total bounds
  lb = VectorT::Zero((m+1)*N);
  ub = VectorT::Zero((m+1)*N);
  Ab = MatrixT::Zero((m+1)*N, m*N);

  VectorXf HX0 = H*X0;
  VectorXf delta(N);  // Kb*H*X0 + Kb*X_ref + K_b*D

  for (int k = 0; k < N; k++) {
    delta(k) = HX0((k+1)*n - 1) + X_ref((k+2)*n - 1) + D((k+1)*n - 1);
  }

  lb.block(0, 0, N, 1) = a - delta;
  ub.block(0, 0, N, 1) = b - delta;

  for (int k = 0; k < N; k++) {
    lb.block(N + k*m, 0, m, 1) = c - U_ref.block((k+1)*m, 0, m, 1);
    ub.block(N + k*m, 0, m, 1) = d - U_ref.block((k+1)*m, 0, m, 1);
  }

  MatrixT KbG(N, m*N);
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < m*N; j++) {
      KbG(i, j) = G((i+1)*n - 1, j);
    }
  }
  Ab.block(0, 0, N, m*N) = KbG;
  Ab.block(N, 0, m*N, m*N) = MatrixT::Identity(m*N, m*N);
}


template <class VectorT, class T>
bool MPCNode::solve_qp(
  const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &P,
  const VectorT &q,
  const VectorT &lb,
  const VectorT &ub,
  const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &Ab,
  VectorT &solution)
{
  typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> MatrixT;
  int n = q.rows();
  int m = ub.rows();

  // Workspace structures
  OSQPWorkspace *work;
  OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
  OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

  // Populate data
  if (data) {
    MatrixT P_tri = P.template triangularView<Eigen::Upper>();  // upper part of P
    Eigen::SparseMatrix<T> _P = P_tri.sparseView();
    Eigen::SparseMatrix<T> _A = Ab.sparseView();

    data->P = nullptr;
    data->A = nullptr;

    data->n = n;
    data->m = m;
    OsqpEigen::SparseMatrixHelper::createOsqpSparseMatrix(_P, data->P);
    data->q = const_cast<T*>(q.data());
    OsqpEigen::SparseMatrixHelper::createOsqpSparseMatrix(_A, data->A);
    data->l = const_cast<T*>(lb.data());
    data->u = const_cast<T*>(ub.data());
  }

  // Define solver settings as default
  if (settings) {
    osqp_set_default_settings(settings);
    settings->alpha = 1.0; // Change alpha parameter
    settings->verbose = false;
  }

  // Setup workspace
  c_int setup_flag = osqp_setup(&work, data, settings);

  if (setup_flag != 0) {
    ROS_WARN("[mpc_node] Setup of MPC problem failed");
    string flag_string = "";

    switch (setup_flag) {
      case OSQP_DATA_VALIDATION_ERROR: flag_string = "OSQP_DATA_VALIDATION_ERROR"; break;
      case OSQP_SETTINGS_VALIDATION_ERROR: flag_string = "OSQP_SETTINGS_VALIDATION_ERROR"; break;
      case OSQP_LINSYS_SOLVER_LOAD_ERROR: flag_string = "OSQP_LINSYS_SOLVER_LOAD_ERROR"; break;
      case OSQP_LINSYS_SOLVER_INIT_ERROR: flag_string = "OSQP_LINSYS_SOLVER_INIT_ERROR"; break;
      case OSQP_NONCVX_ERROR: flag_string = "OSQP_NONCVX_ERROR"; break;
      case OSQP_MEM_ALLOC_ERROR: flag_string = "OSQP_MEM_ALLOC_ERROR"; break;
      case OSQP_WORKSPACE_NOT_INIT_ERROR: flag_string = "OSQP_WORKSPACE_NOT_INIT_ERROR"; break;
      default: break;
    }
    ROS_WARN_STREAM("[mpc_node] " << flag_string);
    qp_warm_start_ = false;
    return false;
  }

  // Warm start (FIXME)
  qp_warm_start_ = false;  // FIXME: warm start leads to big issues
  if (qp_warm_start_)
    osqp_warm_start(work, qp_last_primal_.data(), qp_last_dual_.data());

  // Solve Problem
  c_int solve_ret = 0;  // return code of the solving operation (0: no error)
  solve_ret = osqp_solve(work);

  if (solve_ret != 0 || work->info->status_val != OSQP_SOLVED) {
    ROS_WARN("[mpc_node] Couldn't solve MPC problem");
    string flag_string = "";

    switch (work->info->status_val) {
      case OSQP_DUAL_INFEASIBLE_INACCURATE: flag_string = "OSQP_DUAL_INFEASIBLE_INACCURATE"; break;
      case OSQP_PRIMAL_INFEASIBLE_INACCURATE: flag_string = "OSQP_PRIMAL_INFEASIBLE_INACCURATE"; break;
      case OSQP_SOLVED_INACCURATE: flag_string = "OSQP_SOLVED_INACCURATE"; break;
      case OSQP_SOLVED: flag_string = "OSQP_SOLVED"; break;
      case OSQP_MAX_ITER_REACHED: flag_string = "OSQP_MAX_ITER_REACHED"; break;
      case OSQP_PRIMAL_INFEASIBLE: flag_string = "OSQP_PRIMAL_INFEASIBLE"; break;
      case OSQP_DUAL_INFEASIBLE: flag_string = "OSQP_DUAL_INFEASIBLE"; break;
      case OSQP_SIGINT: flag_string = "OSQP_SIGINT"; break;
      case OSQP_NON_CVX: flag_string = "OSQP_NON_CVX"; break;
      case OSQP_UNSOLVED: flag_string = "OSQP_UNSOLVED"; break;
      default: break;
    }
    ROS_WARN_STREAM("[mpc_node] " << flag_string);
    qp_warm_start_ = false;
    return false;
  }

  // Parse solution
  solution = VectorT(n);
  qp_last_primal_ = VectorT(n);
  qp_last_dual_ = VectorT(n);

  for (int k = 0; k < n; k++) {
    solution(k) = work->solution->x[k];
    qp_last_primal_(k) = work->solution->x[k];
    qp_last_dual_(k) = work->solution->y[k];
  }

  qp_warm_start_ = true;

  // Cleanup
  if (data) {
    if (data->A) c_free(data->A);
    if (data->P) c_free(data->P);
    c_free(data);
  }
  if (settings) c_free(settings);
}


bool MPCNode::compute_control(
  float &desired_speed,
  float last_desired_speed,
  std::vector<float> &command,
  geometry_msgs::PoseArray &expected_traj)
{
  if (path_.poses.size() < 2) {
    ROS_WARN("[mpc_node] Path size < 2");
    return false;
  }

  // Parse current state
  geometry_msgs::Point current_position;
  current_position.x = state_[0];
  current_position.y = state_[1];
  current_position.z = state_[2];

  int n = state_.size();
  int m = last_control_.size();

  // Store the original path for convenience
  int size_path = path_.poses.size();
  vector<geometry_msgs::Pose> orig_path(size_path);

  for (int k = 0; k < size_path; k++) {
    orig_path[k] = path_.poses[k].pose;
  }

  // Get path with suitable length and resolution
  float spatial_resolution = desired_speed * time_horizon_ / nbr_steps_;  // spatial resolution
  float time_resolution = time_horizon_ / nbr_steps_;

  vector<geometry_msgs::Pose> new_path = adapt_path(orig_path, current_position,
    nbr_steps_, spatial_resolution, time_resolution, desired_speed);

  // Fill intial points
  VectorXf x0(n), X0(n);  // initial state, and initial offset to the reference
  VectorXf u_m1(m);       // last control applied to the robot

  for (int k = 0; k < n; k++) x0(k) = state_[k];
  for (int k = 0; k < m; k++) u_m1(k) = last_control_[k];

  // Fill reference points for MPC optimisation
  int N = nbr_steps_;
  VectorXf X_ref, U_ref;

  if (!fill_ref_pts(N, n, m, new_path, desired_speed, last_desired_speed, robot_model_,
    X_ref, U_ref))
  {
    ROS_WARN("[mpc_node] Reference trajectory could not be filled");
    return false;
  }

  modulo_ref_state(X_ref, x0, n);  // change reference orientation to prevent modulo discontinuity

  X0 = x0 - X_ref.block(0, 0, n, 1);
  VectorXf x0_ref = X_ref.block(0, 0, n, 1);  // first state reference
  VectorXf u0_ref = U_ref.block(0, 0, m, 1);  // first control reference

  // Build MPC objects
  MatrixXf G, H;  // to express X with respect to U and X0
  VectorXf D;     // idem
  MatrixXf L, M;  // to compute quadratic parts of the cost function
  VectorXf V;     // to compute linear parts of the cost function
  MatrixXf LG;    // product of L and G

  fill_L(tuning_params_.P, tuning_params_.Q_x, N, L);
  fill_M(tuning_params_.R_u, tuning_params_.R_delta, N, M);
  fill_V(u0_ref, u_m1, tuning_params_.R_delta, N, V);

  if (ltv_mpc_) {
    // LTV MPC: linearise at each reference point
    fill_ltv_G_H_D(X_ref, U_ref, N, time_resolution, spatial_resolution, G, H, D);
    LG = L * G;
  } else {
    // LTI MPC: only linearise at first reference point
    MatrixXf Ad, Bd;  // discretised model
    robot_model_.get_lin_discr_matrices(x0_ref, u0_ref, Ad, Bd, time_resolution);

    fill_lti_G(Ad, Bd, N, G);
    fill_lti_H(Ad, N, H);
    fill_lti_LG(G, tuning_params_.P, tuning_params_.Q_x, n, m, N, LG);
    D = VectorXf::Zero(n*N);
  }

  // Build MPC bounds objects
  VectorXf lb, ub;  // lower and upper bounds
  MatrixXf Ab;  // multiplicative factor in front of U in the bound inequalities

  fill_bounds_objs(bounds_, n, N, X0, X_ref, U_ref, G, H, D, lb, ub, Ab);

  // Solve MPC Quadratic Program
  MatrixXf P = 2*(G.transpose()*LG + M);
  VectorXf q = 2*LG.transpose()*(H*X0 + D) + V;
  VectorXf solution;

  bool solved = solve_qp(P, q, lb, ub, Ab, solution);

  if (!solved)
    return false;

  // Prepare command output
  command.resize(m);
  for (int k = 0; k < m; k++)
    command[k] = solution(k) + U_ref(k);

  // Compute expected controlled trajectory
  VectorXf X = G*solution + H*X0 + D + X_ref.block(n, 0, n*N, 1);
  expected_traj.poses.resize(N);

  for (int k = 0; k < N; k++) {
    geometry_msgs::Pose pose;
    pose.position.x = X(k*n);
    pose.position.y = X(k*n + 1);
    pose.position.z = X(k*n + 2);
    to_quaternion(X(k*n + 3), X(k*n + 4), X(k*n + 5), pose.orientation);

    expected_traj.poses[k] = pose;
  }


  return true;
}

}  // namespace mfcpp
