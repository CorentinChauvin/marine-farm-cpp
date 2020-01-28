/**
 * @file
 *
 * \brief  Declaration of a node to generate a fake trajectory and evaluate the
 *         performance of the control.
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#ifndef CONTROL_TESTER_HPP
#define CONTROL_TESTER_HPP

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <string>


namespace mfcpp {

/**
 * \brief  Class to test performance of control
 *
 * Generates a trajectory to follow, and measure how well the robot is following
 * it.
 */
class ControlTesterNode {
  public:
    ControlTesterNode();
    ~ControlTesterNode();

    /**
     * \brief  Runs the node
     */
    void run_node();

  private:
    // Private members
    ros::NodeHandle nh_;  ///<  Node handler
    ros::Subscriber odom_sub_;  ///<  Subscriber for robot odometry
    ros::Publisher path_pub_;   ///<  Publisher for a path

    nav_msgs::Odometry odom_;  ///<  Last odometry message received
    bool odom_received_;       ///<  Whether odometry has been received
    nav_msgs::Path path_;      ///<  Path to publish
    bool path_loaded_;         ///<  Whether the path was loaded with success


    /// \name  ROS parameters
    ///@{
    float main_freq_;   ///<  Frequency of the main loop
    ///@}


    /**
     * \brief  Initialises the node and its parameters
     */
    void init_node();

    /**
     * \brief  Loads a path from a text file
     *
     * The text file contains waypoints that are then interpolated by a spline.
     *
     * \param[in]  file_name    Relative path of the file containing the waypoints
     * \param[in]  resolution   Spatial resolution of the path
     * \param[out] path         Loaded path
     */
    void load_path(std::string file_name, float resolution, nav_msgs::Path &path);

    /**
     * \brief  Callback for odometry
     */
    void odom_cb(const nav_msgs::Odometry msg);

};


}  // namespace mfcpp

#endif
