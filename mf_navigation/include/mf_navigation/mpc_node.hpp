/**
 * @file
 *
 * \brief  Declaration of a node for Model Predictive Control of an underwater
 *         robot
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#ifndef MPC_NODE_HPP
#define MPC_NODE_HPP

#include "mf_robot_model/robot_model.hpp"
#include "mf_common/Float32Array.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

namespace mfcpp {

/**
 * \brief  Node for Model Predictive Control of a robot
 *
 * `MatrixT` template can either be `Eigen::MatrixXf` or `Eigen::MatrixXd`.
 * `VectorT` template can either be `Eigen::VectorXf` or `Eigen::VectorXd`.
 */
class MPCNode {
  public:
    MPCNode();
    ~MPCNode();

    /**
     * \brief  Runs the node
     */
    void run_node();

  private:
    /**
     * \brief  MPC tuning parameters
     */
    struct MPCTuningParameters
    {
      Eigen::MatrixXf P;        ///<  Penalty on the last state error
      Eigen::MatrixXf Q_x;      ///<  Penalty on the intermediary states errors
      Eigen::MatrixXf R_u;      ///<  Penalty on the control input error
      Eigen::MatrixXf R_delta;  ///<  Penalty on the control change rate
    };

    /**
     * \brief  Bounds on the MPC problem
     */
    struct MPCBounds
    {
      double delta_m;  // bound on delta_m
      std::vector<double> input;  // bounds on the control input
    };

    // Private members
    ros::NodeHandle nh_;          ///<  Node handler (for topics and services)
    ros::Subscriber path_sub_;    ///<  Subscriber for the desired path
    ros::Subscriber state_sub_;   ///<  Subscriber for the current robot state
    ros::Publisher command_pub_;  ///<  Publisher for the computed command
    ros::Publisher expected_traj_pub_;  ///<  Publisher for the expected controlled trajectory
    tf2_ros::Buffer tf_buffer_;   ///<  Buffer for tf2
    tf2_ros::TransformListener tf_listener_;  ///<  Transform listener for tf2

    nav_msgs::Path path_;     ///<  Path to follow
    bool path_received_;      ///<  Whether a new path has been received
    bool state_received_;     ///<  Whether the robot state has ever been received
    RobotModel robot_model_;  ///<  Robot model
    std::vector<float> state_;           ///<  Current robot state
    std::vector<float> last_control_;    ///<  Last control applied to the robot
    MPCTuningParameters tuning_params_;  ///<  MPC tuning parameters
    MPCBounds bounds_;  ///<  Bounds for the MPC

    Eigen::VectorXf qp_last_primal_;  ///<  Last primal solution of the Quadratic Problem
    Eigen::VectorXf qp_last_dual_;    ///<  Last dual solution of the Quadratic Problem
    bool qp_warm_start_;              ///<  Whether the QP can start warm

    /// \name  ROS parameters
    ///@{
    float main_freq_;  ///<  Frequency of the main loop
    std::string fixed_frame_;  ///<  Fixed frame

    float desired_speed_;  ///<  Desired speed (m/s) of the robot
    float last_desired_speed_;  ///<  Last desired speed (m/s) of the robot
    float time_horizon_;   ///<  Time horizon (s) for the MPC prediction
    int nbr_steps_;        ///<  Number of steps for the MPC prediction
    bool disable_vbs_;     ///<  Whether to disable Variable Buoyancy System (VBS)
    bool ltv_mpc_;         ///<  Whether to use the Linear Time Varying (LTV) version of MPC
    ///@}

    /**
     * \brief  Initialises the node and its parameters
     */
    void init_node();

    /**
     * \brief  Generates a path of the right size for MPC prediction
     *
     * The path will have as many poses as steps for MPC, with a desired spatial
     * resolution. The desired speed and size of the path are maintained, so if
     * the desired spatial resolution leads to a too long path, the resolution
     * is decreased to fit. If so, the time resolution is also shrinked (ie
     * the path duration is smaller) and the desired speed is decreased.
     *
     * \param[in]  orig_path              Path provided by the path planner
     * \param[in]  current_position       Current position of the robot
     * \param[in]  nbr_steps              Number of MPC prediction steps
     * \param[in,out] spatial_resolution  Distance between two MPC steps
     * \param[in,out] time_resolution     Time between two MPC steps
     * \param[in,out] desired_speed       Desired speed
     * \return  Path of size nbr_steps+1
     */
    std::vector<geometry_msgs::Pose> adapt_path(
      const std::vector<geometry_msgs::Pose> &orig_path,
      const geometry_msgs::Point &current_position,
      int nbr_steps,
      float &spatial_resolution,
      float &time_resolution,
      float &desired_speed
    );

    /**
     * \brief  Fills reference points for MPC optimisation
     *
     * \param[in]  N                    Number of steps for the MPC prediction
     * \param[in]  n                    Dimension of the state vector
     * \param[in]  m                    Dimension of the control vector
     * \param[in]  path                 Reference path to follow (of size N+1)
     * \param[in]  desired_speed        Desired speed (m/s) of the robot
     * \param[in]  last_desired_speed   Last desired speed (m/s) of the robot
     * \param[in]  robot_model          Robot model
     * \param[out] X_ref                Reference state to fill (x0_ref, ..., xN_ref)
     * \param[out] U_ref                Reference output to fill (u0_ref, ..., uN_ref)
     * \return  Whether the points could be filled
     */
    template <class VectorT>
    bool fill_ref_pts(
      int N, int n, int m,
      const std::vector<geometry_msgs::Pose> &path,
      float desired_speed,
      float last_desired_speed,
      const RobotModel &robot_model,
      VectorT &X_ref,
      VectorT &U_ref
    );

    /**
     * \brief  Fills the G and H matrices used to express X with respect to U and X0
     *
     * Linear Time Varying MPC version.
     *
     * \param[in]  X_ref  State reference
     * \param[in]  U_ref  Control reference
     * \param[in]  N      Number of steps for the MPC prediction
     * \param[in]  dt     Sampling time
     * \param[in]  ds     Spatial resolution of reference path
     * \param[out] G      G matrix
     * \param[out] H      H matrix
     * \param[out] D      D vector
     */
    template <class VectorT, class MatrixT>
    void fill_ltv_G_H_D(const VectorT &X_ref, const VectorT &U_ref,
      int N, float dt, float ds,
      MatrixT &G, MatrixT &H, VectorT &D);

    /**
     * \brief  Fills the G matrix used to express X with respect to U and X0
     *
     * Linear Time Invariant MPC version.
     *
     * \param[in]  Ad  Discretised model matrix
     * \param[in]  Bd  Discretised model matrix
     * \param[in]  N   Number of steps for the MPC prediction
     * \param[out] G   G matrix
     */
    template <class MatrixT>
    void fill_lti_G(const MatrixT &Ad, const MatrixT &Bd, int N, MatrixT &G);

    /**
     * \brief  Fills the H matrix used to express X with respect to U and X0
     *
     * Linear Time Invariant MPC version
     *
     * \param[in]  Ad  Discretised model matrix
     * \param[in]  N   Number of steps for the MPC prediction
     * \param[out] H   H matrix
     */
    template <class MatrixT>
    void fill_lti_H(const MatrixT &Ad, int N, MatrixT &H);

    /**
     * \brief  Fills the L matrix used for quadratic cost wrt state error
     *
     * \param[in]  P    Penalty on the last state error
     * \param[in]  Q_x  Penalty on the intermediary states errors
     * \param[in]  N    Number of steps for the MPC prediction
     * \param[out] L    L matrix
     */
    template <class MatrixT>
    void fill_L(const MatrixT &P, const MatrixT &Q_x, int N, MatrixT &L);

    /**
     * \brief  Fills the M matrix used for quadratic cost wrt control input error
     *
     * \param[in]  R_u      Penalty on the control input error
     * \param[in]  R_delta  Penalty on the control change rate
     * \param[in]  N        Number of steps for the MPC prediction
     * \param[out] M        M matrix
     */
    template <class MatrixT>
    void fill_M(const MatrixT &R_u, const MatrixT &R_delta, int N, MatrixT &M);

    /**
     * \brief  Fills the V matrix used for the linear cost wrt control input error
     *
     * \param[in]  control_ref   Reference control
     * \param[in]  last_control  Last control applied to the robot
     * \param[in]  R_delta       Penalty on the control change rate
     * \param[in]  N             Number of steps for the MPC prediction
     * \param[out] V             V matrix
     */
    template <class VectorT, class MatrixT>
    void fill_V(VectorT control_ref, VectorT last_control,
      const MatrixT &R_delta, int N, VectorT &V);

    /**
     * \brief  Fills the product of L and G matrices
     *
     * Optimisation in the Linear Time Invariant MPC case
     *
     * \param[in]  G    G matrix
     * \param[in]  P    Penalty on the last state error
     * \param[in]  Q_x  Penalty on the intermediary states errors
     * \param[in]  n    Size of the state
     * \param[in]  m    Size of the input
     * \param[in]  N    Number of steps for the MPC prediction
     * \param[out] LG   LG matrix
     */
    template <class MatrixT>
    void fill_lti_LG(const MatrixT &G, const MatrixT &P, const MatrixT &Q_x, int n,
      int m, int N, MatrixT &LG);

    /**
     * \brief  Fills the different bounds objects
     *
     * \param[in]  bounds  Bounds of the MPC problem
     * \param[in]  n       Size of the state
     * \param[in]  N       Number of steps for the MPC prediction
     * \param[in]  X0      Initial state
     * \param[in]  X_ref   Reference state (X_0_ref, ..., X_N_ref)
     * \param[in]  G       Used to express X with respect to U and X0
     * \param[in]  H       Used to express X with respect to U and X0
     * \param[out] lb      Lower bound on Ab*U
     * \param[out] ub      Upper bound on Ab*U
     * \param[out] Ab      Multiplicative factor in front of U in the bound inequalities
     */
    template <class VectorT, class MatrixT>
    void fill_bounds_objs(
      const MPCBounds &bounds,
      int n, int N,
      const VectorT &X0, const VectorT &X_ref,
      const MatrixT &G, const MatrixT &H,
      VectorT &lb, VectorT &ub, MatrixT &Ab
    );

    /**
     * \brief  Solves a Quadratic Program
     *
     *
     */
    template <class VectorT, class T>
    bool solve_qp(
      const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &P,
      const VectorT &q,
      const VectorT &lb,
      const VectorT &ub,
      const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &Ab,
      VectorT &solution
    );

    /**
     * \brief  Computes the control signal to send to the robot
     *
     * The desired speed might be changed if the robot is close to the end of
     * the path.
     *
     * \param[in,out] desired_speed       Desired speed (m/s) of the robot
     * \param[in]     last_desired_speed  Last desired speed
     * \param[out]    command             Computed control input to apply
     * \param[out]    expected_traj       Expected controlled trajectory
     *
     * \return  Whether the control could be computed
     */
    bool compute_control(
      float &desired_speed,
      float last_desired_speed,
      std::vector<float> &command,
      geometry_msgs::PoseArray &expected_traj
    );

    /**
     * \brief  Callback for the desired path
     */
    void path_cb(const nav_msgs::Path msg);

    /**
     * \brief  Callback for the current robot state
     */
    void state_cb(const mf_common::Float32Array msg);

};

}  // namespace mfcpp

#endif
