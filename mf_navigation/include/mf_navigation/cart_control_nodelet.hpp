/**
 * @file
 *
 * \brief  Declaration of a nodelet for cheated cartesian control of a robot
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#ifndef CART_CONTROL_NODELET_HPP
#define CART_CONTROL_NODELET_HPP

#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <csignal>

namespace mfcpp {

/**
 * \brief  Nodelet for cheated cartesian control of a robot
 *
 * The nodelet just teleports the robot along a given path, not taking into
 * account any physical constraint.
 */
class CartControlNodelet: public nodelet::Nodelet {
  public:
    CartControlNodelet();
    ~CartControlNodelet();

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
    ros::Publisher pose_pub_;     ///<  Publisher for the output pose
    nav_msgs::Path path_;         ///<  Desired path
    bool path_received_;          ///<  Whether a new path has been received
    bool path_processed_;         ///<  Whether the path has been processed
    bool path_executed_;          ///<  Whether the previous path has been executed
    std::vector<geometry_msgs::Pose> waypoints_;  ///<  Consecutive poses that the robot will be assigned to
    int idx_waypoints_;   ///<  Current position in the waypoints list

    /// \name  ROS parameters
    ///@{
    float main_freq_;     ///<  Frequency of the main loop
    float robot_speed_;   ///<  Desired robot speed
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
     *
     * \note  The frame in which the path is expressed should match with the fixed
     *        frame of the robot simulator.
     */
    void path_cb(const nav_msgs::PathConstPtr msg);

    /**
     * \brief  Computes the poses that the controller will assign to the robot
     *
     * \param[in]  path       Path to follow
     * \param[in]  speed      Desired speed of the robot
     * \param[in]  dt         Time interval between two poses
     * \param[out] waypoints  List of computed waypoints
     */
    void compute_waypoints(const nav_msgs::Path &path, float speed, float dt,
      std::vector<geometry_msgs::Pose> &waypoints) const;


};

}  // namespace mfcpp

#endif
