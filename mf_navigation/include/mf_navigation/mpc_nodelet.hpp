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
#include <nav_msgs/Path.h>
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

    nav_msgs::Path path_;         ///<  Desired path
    bool path_received_;          ///<  Whether a new path has been received
    RobotModel robot_model_;      ///<  Robot model
    Eigen::MatrixXd Ad_;  ///<  First matrix of the discretised linearised system
    Eigen::MatrixXd Bd_;  ///<  Second matrix of the discretised linearised system

    /// \name  ROS parameters
    ///@{
    float main_freq_;     ///<  Frequency of the main loop
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
     * \brief  Callback for the desired path
     */
    void path_cb(const nav_msgs::PathConstPtr msg);

};

}  // namespace mfcpp

#endif
