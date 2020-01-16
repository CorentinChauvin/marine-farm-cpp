/**
 * @file
 *
 * \brief  Declaration of a nodelet for Model Predictive Control of an underwater
 *         robot
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#ifndef MPC_NODELET_HPP
#define MPC_NODELET_HPP

#include "mf_robot_model/robot_model.hpp"
#include "mf_common/Float32Array.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <csignal>

namespace mfcpp {

/**
 * \brief  Nodelet for Model Predictive Control of a robot
 *
 * `MatrixT` template can either be `Eigen::MatrixXf` or `Eigen::MatrixXd`.
 * `VectorT` template can either be `Eigen::VectorXf` or `Eigen::VectorXd`.
 */
class MPCNodelet: public nodelet::Nodelet {
  public:
    MPCNodelet();
    ~MPCNodelet();

    /**
     * \brief  Function called at beginning of nodelet execution
     */
    virtual void onInit();

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

    // Static members
    // Note: the timers need to be static since stopped by the SIGINT callback
    static sig_atomic_t volatile b_sigint_;  ///<  Whether SIGINT signal has been received
    static ros::Timer main_timer_;  ///<  Timer callback for the main function

    // Private members
    ros::NodeHandle nh_;          ///<  Node handler (for topics and services)
    ros::NodeHandle private_nh_;  ///<  Private node handler (for parameters)
    ros::Subscriber path_sub_;    ///<  Subscriber for the desired path
    ros::Subscriber state_sub_;   ///<  Subscriber for the current robot state
    tf2_ros::Buffer tf_buffer_;   ///<  Buffer for tf2
    tf2_ros::TransformListener tf_listener_;  ///<  Transform listener for tf2

    nav_msgs::Path path_;     ///<  Desired path
    bool path_received_;      ///<  Whether a new path has been received
    bool state_received_;     ///<  Whether the robot state has ever been received
    RobotModel robot_model_;  ///<  Robot model
    std::vector<float> state_;           ///<  Current robot state
    std::vector<float> last_control_;    ///<  Last control applied to the robot
    MPCTuningParameters tuning_params_;  ///<  MPC tuning parameters
    MPCBounds bounds_;

    /// \name  ROS parameters
    ///@{
    float main_freq_;  ///<  Frequency of the main loop

    float desired_speed_;  ///<  Desired speed (m/s) of the robot
    float last_desired_speed_;  ///<  Last desired speed (m/s) of the robot
    float time_horizon_;   ///<  Time horizon (s) for the MPC prediction
    int nbr_steps_;        ///<  Number of steps for the MPC prediction (fixed in CVXGEN)
    ///@}

    /**
     * \brief  Main callback which is called by a timer
     *
     * \param timer_event  Timer event information
     */
    void main_cb(const ros::TimerEvent &timer_event);

    /**
     * \brief  SINGINT (Ctrl+C) callback to stop the nodelet properly
     */
    static void sigint_handler(int s);

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
     * \param[in]  path                 Reference path to follow
     * \param[in]  desired_speed        Desired speed (m/s) of the robot
     * \param[in]  last_desired_speed   Last desired speed (m/s) of the robot
     * \param[in]  robot_model          Robot model
     * \param[out] X_ref                Reference state to fill
     * \param[out] U_ref                Reference output to fill
     */
    template <class VectorT>
    void fill_ref_pts(
      int N, int n, int m,
      const std::vector<geometry_msgs::Pose> &path,
      float desired_speed,
      float last_desired_speed,
      const RobotModel &robot_model,
      VectorT &X_ref,
      VectorT &U_ref
    );

    /**
     * \brief  Fills the G matrix used to express X with respect to U and X0
     *
     * \param[in]  Ad  Discretised model matrix
     * \param[in]  Bd  Discretised model matrix
     * \param[in]  N   Number of steps for the MPC prediction
     * \param[out] G   G matrix
     */
    template <class MatrixT>
    void fill_G(const MatrixT &Ad, const MatrixT &Bd, int N, MatrixT &G);

    /**
     * \brief  Fills the H matrix used to express X with respect to U and X0
     *
     * \param[in]  Ad  Discretised model matrix
     * \param[in]  N   Number of steps for the MPC prediction
     * \param[out] H   H matrix
     */
    template <class MatrixT>
    void fill_H(const MatrixT &Ad, int N, MatrixT &H);

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
     * \param[in]  desired_speed        Current desired speed
     * \param[in]  last_desired_speed   Last desired speed
     * \param[in]  R_delta              Penalty on the control change rate
     * \param[in]  N                    Number of steps for the MPC prediction
     * \param[out] V                    V matrix
     */
    template <class VectorT, class MatrixT>
    void fill_V(float desired_speed, float last_desired_speed,
      const MatrixT &R_delta, int N, VectorT &V);

    /**
     * \brief  Fills the different bounds objects
     *
     * \param[in]  bounds  Bounds of the MPC problem
     * \param[in]  n       Size of the state
     * \param[in]  N       Number of steps for the MPC prediction
     * \param[in]  X0      Initial state
     * \param[in]  X_ref   Reference state
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
     * \brief  Computes the control signal to send to the robot
     *
     * The desired speed might be changed if the robot is close to the end of
     * the path.
     *
     * \param[in]     path                Total desired path to follow
     * \param[in]     current_state       Current state of the robot
     * \param[in]     last_control        Last control applied to the robot
     * \param[in]     robot_model         Robot model
     * \param[in]     tuning_params       MPC tuning parameters
     * \param[in,out] desired_speed       Desired speed (m/s) of the robot
     * \param[in]     last_desired_speed  Last desired speed
     * \param[in]     time_horizon        Time horizon (s) for the MPC prediction
     * \param[in]     nbr_steps           Number of steps for the MPC prediction
     * \param[in]     bounds              Bounds of the MPC problem
     */
    void compute_control(
      const nav_msgs::Path &path,
      const std::vector<float> &current_state,
      const std::vector<float> &last_control,
      const RobotModel &robot_model,
      const MPCTuningParameters &tuning_params,
      float &desired_speed,
      float last_desired_speed,
      float time_horizon,
      int nbr_steps,
      const MPCBounds &bounds
    );

    /**
     * \brief  Callback for the desired path
     */
    void path_cb(const nav_msgs::PathConstPtr msg);

    /**
     * \brief  Callback for the current robot state
     */
    void state_cb(const mf_common::Float32Array msg);

};

}  // namespace mfcpp

#endif
