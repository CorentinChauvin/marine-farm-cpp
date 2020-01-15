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
    Eigen::MatrixXd Ad_;  ///<  First matrix of the discretised linearised system
    Eigen::MatrixXd Bd_;  ///<  Second matrix of the discretised linearised system
    std::vector<float> state_;  ///<  Current robot state

    /// \name  ROS parameters
    ///@{
    float main_freq_;  ///<  Frequency of the main loop

    float nominal_speed_;  ///<  Nominal speed (m/s) of the robot
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
     * the path duration is smaller).
     *
     * \param[in]  orig_path              Path provided by the path planner
     * \param[in]  current_position       Current position of the robot
     * \param[in]  nbr_steps              Number of MPC prediction steps
     * \param[in,out] spatial_resolution  Distance between two MPC steps
     * \param[in,out] time_resolution     Time between two MPC steps
     */
    std::vector<geometry_msgs::Pose> adapt_path(
      const std::vector<geometry_msgs::Pose> &orig_path,
      const geometry_msgs::Point &current_position,
      int nbr_steps,
      float &spatial_resolution,
      float &time_resolution
    );

    /**
     * \brief  Computes the control signal to send to the robot
     *
     * \param path           Total desired path to follow
     * \param current_state  Current state of the robot
     * \param nominal_speed  Nominal speed (m/s) of the robot
     * \param time_horizon   Time horizon (s) for the MPC prediction
     * \param nbr_steps      Number of steps for the MPC prediction
     */
    void compute_control(
      const nav_msgs::Path &path,
      const std::vector<float> &current_state,
      float nominal_speed,
      float time_horizon,
      int nbr_steps
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
