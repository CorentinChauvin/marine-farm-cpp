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
    return vector<geometry_msgs::Pose>(nbr_steps, orig_path[size_path-1]);
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

  new_path.erase(new_path.begin());  // remove first element of the path (current pose)

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
  if (N < 1 || n < 1 || m < 1 || path.size() != N) {
    ROS_WARN("[mpc_node] Wrong input dimensions in fill_ref_pts:");
    ROS_WARN("[mpc_node] path size: %d ; N=%d", (int) path.size(), N);
    return false;
  }

  X_ref = VectorT::Zero(n * N);
  U_ref = VectorT::Zero(m * N);

  for (int k = 0; k < N; k++) {
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

  for (int k = 0; k < N; k++) {
    U_ref(k*m) = desired_prop_speed;
  }

  return true;
}


template <class MatrixT>
void MPCNode::fill_G(const MatrixT &Ad, const MatrixT &Bd, int N, MatrixT &G)
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
void MPCNode::fill_H(const MatrixT &Ad, int N, MatrixT &H)
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
void MPCNode::fill_LG(const MatrixT &G, const MatrixT &P, const MatrixT &Q_x,
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
  const VectorT &X0, const VectorT &X_ref,
  const MatrixT &G, const MatrixT &H,
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
  VectorXf delta(N);  // Kb*H*X0 - Kb*X_ref

  for (int k = 0; k < N; k++) {
    delta(k) = HX0((k+1)*n - 1) - X_ref((k+1)*n - 1);
  }

  lb.block(0, 0, N, 1) = a - delta;
  ub.block(0, 0, N, 1) = b - delta;

  for (int k = 0; k < N; k++) {
    lb.block(N + k*m, 0, m, 1) = c;
    ub.block(N + k*m, 0, m, 1) = d;
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


  // cout << "P:\n" << P << endl;
  // cout << "q:\n" << q << endl;
  // cout << "lb:\n" << lb << endl;
  // cout << "ub:\n" << ub << endl;
  // cout << "Ab:\n" << Ab << endl;

  // Debug data
  // n = 2;
  // m = 3;
  // MatrixXf P_(n, n), Ab_(m, n);
  // VectorXf lb_(m), ub_(m), q_(n);
  //
  // P_ << 5, 1, 1, 2;
  // q_ << 1, 1;
  // lb_ << 1, 0, 0;
  // ub_ << 1, 0.7, 0.7;
  // Ab_ << 1, 1, 1, 0, 0, 1;


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

    // TODO: remove later
    switch (setup_flag) {
      case OSQP_DATA_VALIDATION_ERROR: cout << "OSQP_DATA_VALIDATION_ERROR" << endl; break;
      case OSQP_SETTINGS_VALIDATION_ERROR: cout << "OSQP_SETTINGS_VALIDATION_ERROR" << endl; break;
      case OSQP_LINSYS_SOLVER_LOAD_ERROR: cout << "OSQP_LINSYS_SOLVER_LOAD_ERROR" << endl; break;
      case OSQP_LINSYS_SOLVER_INIT_ERROR: cout << "OSQP_LINSYS_SOLVER_INIT_ERROR" << endl; break;
      case OSQP_NONCVX_ERROR: cout << "OSQP_NONCVX_ERROR" << endl; break;
      case OSQP_MEM_ALLOC_ERROR: cout << "OSQP_MEM_ALLOC_ERROR" << endl; break;
      case OSQP_WORKSPACE_NOT_INIT_ERROR: cout << "OSQP_WORKSPACE_NOT_INIT_ERROR" << endl; break;
      default: break;
    }

    return false;
  }


  // Solve Problem
  c_int solve_ret = 0;  // return code of the solving operation (0: no error)
  solve_ret = osqp_solve(work);

  if (solve_ret != 0 || work->info->status_val != OSQP_SOLVED) {
    ROS_WARN("[mpc_node] Couldn't solve MPC problem");

    // TODO: remove later
    switch (work->info->status_val) {
      case OSQP_DUAL_INFEASIBLE_INACCURATE: cout << "OSQP_DUAL_INFEASIBLE_INACCURATE" << endl; break;
      case OSQP_PRIMAL_INFEASIBLE_INACCURATE: cout << "OSQP_PRIMAL_INFEASIBLE_INACCURATE" << endl; break;
      case OSQP_SOLVED_INACCURATE: cout << "OSQP_SOLVED_INACCURATE" << endl; break;
      case OSQP_SOLVED: cout << "OSQP_SOLVED" << endl; break;
      case OSQP_MAX_ITER_REACHED: cout << "OSQP_MAX_ITER_REACHED" << endl; break;
      case OSQP_PRIMAL_INFEASIBLE: cout << "OSQP_PRIMAL_INFEASIBLE" << endl; break;
      case OSQP_DUAL_INFEASIBLE: cout << "OSQP_DUAL_INFEASIBLE" << endl; break;
      case OSQP_SIGINT: cout << "OSQP_SIGINT" << endl; break;
      case OSQP_NON_CVX: cout << "OSQP_NON_CVX" << endl; break;
      case OSQP_UNSOLVED: cout << "OSQP_UNSOLVED" << endl; break;
      default: break;
    }

    cout << "lb: \n" << lb.transpose() << endl;
    cout << "ub: \n" << ub.transpose() << endl;

    return false;
  }

  // Parse solution
  solution = VectorT(n);
  for (int k = 0; k < n; k++)
    solution(k) = work->solution->x[k];

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

  clock_t start = clock();
  cout << "---" << endl;
  cout << "desired speed: " << desired_speed << endl;

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

  // Fill reference points for MPC optimisation
  int N = nbr_steps_;
  VectorXf X_ref, U_ref;

  if (!fill_ref_pts(N, n, m, new_path, desired_speed, last_desired_speed, robot_model_,
    X_ref, U_ref))
  {
    ROS_WARN("[mpc_node] Reference trajectory could not be filled");
    return false;
  }

  // Fill intial points
  VectorXf x0(n), X0(n);  // initial state, and initial offset to the reference
  VectorXf u_m1(m);       // last control applied to the robot
  VectorXf u0_ref(m);     // first control reference

  for (int k = 0; k < n; k++) x0(k) = state_[k];
  for (int k = 0; k < m; k++) u_m1(k) = last_control_[k];

  X0 = x0 - X_ref.block(0, 0, n, 1);
  u0_ref = U_ref.block(0, 0, m, 1);

  // Build MPC objects
  MatrixXf Ad, Bd;  // discretised model
  MatrixXf G, H;    // to express X with respect to U and X0
  MatrixXf L, M;    // to compute quadratic partts of the cost function
  VectorXf V;       // to compute linear parts of the cost function
  MatrixXf LG;      // product of L and G
  // VectorXf origin_x = VectorXf::Zero(n);  // origin for linearisation
  // VectorXf origin_u = VectorXf::Zero(m);
  VectorXf origin_x = X_ref.block(0, 0, n, 1);  // origin for linearisation
  VectorXf origin_u = U_ref.block(0, 0, m, 1);

  robot_model_.get_lin_discr_matrices(origin_x, origin_u, Ad, Bd, time_resolution);
  fill_G(Ad, Bd, N, G);
  fill_H(Ad, N, H);
  fill_L(tuning_params_.P, tuning_params_.Q_x, N, L);
  fill_M(tuning_params_.R_u, tuning_params_.R_delta, N, M);
  fill_V(u0_ref, u_m1, tuning_params_.R_delta, N, V);
  fill_LG(G, tuning_params_.P, tuning_params_.Q_x, n, m, N, LG);

  // Build MPC bounds objects
  VectorXf lb, ub;  // lower and upper bounds
  MatrixXf Ab;  // multiplicative factor in front of U in the bound inequalities

  fill_bounds_objs(bounds_, n, N, X0, X_ref, G, H, lb, ub, Ab);

  // Solve MPC Quadratic Program
  MatrixXf P = 2*(G.transpose()*LG + M);
  VectorXf q = 2*LG.transpose()*(H*X0) + V;
  VectorXf solution;

  // TODO: to remove
  // Eigen::LLT<Eigen::MatrixXf> lltOfA(P); // compute the Cholesky decomposition of A
  // if(lltOfA.info() == Eigen::NumericalIssue)
  //   ROS_WARN("Possibly non semi-positive definite matrix!");

  bool solved = solve_qp(P, q, lb, ub, Ab, solution);

  if (!solved)
    return false;

  cout << "Total time: " << (clock() - start) / (double)CLOCKS_PER_SEC << endl;

  // Prepare command output
  command.resize(m);
  for (int k = 0; k < m; k++)
    command[k] = solution(k) + U_ref(k);

  // TODO: to remove
  VectorXf command_vec = (solution + U_ref).block(0, 0, m, 1);
  cout << "Command to apply: " << command_vec.transpose() << endl;

  // Compute expected controlled trajectory
  VectorXf X = G*solution + H*X0 + X_ref;
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
