/**
 * @file
 *
 * \brief  Declaration of a base class node managing high level planning logic
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#ifndef PLANNING_LOGIC_HPP
#define PLANNING_LOGIC_HPP

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <string>
#include <vector>


namespace mfcpp {

/**
 * \brief  Base class for a node managing high level planning logic
 *
 * The planner alternates between hard-coded path for transitions between lines,
 * and informative path planner to survey a wall.
 */
class PlanningLogic {
  public:
    PlanningLogic();
    ~PlanningLogic();

    /**
     * \brief  Runs the node
     */
    void run_node();

  private:
    // Private members
    ros::NodeHandle nh_;  ///<  Node handler
    ros::Subscriber odom_sub_;  ///<  Subscriber for robot's odometry
    ros::Publisher path_pub_;   ///<  Publisher for a path
    ros::ServiceClient disable_planner_client_;  ///<  Client for disabling the path planner
    ros::ServiceClient enable_planner_client_;   ///<  Client for enabling the path planner

    nav_msgs::Odometry odom_;  ///<  Odometry of the robot
    nav_msgs::Path path_;  ///<  Path to publish
    bool odom_received_;   ///<  Whether the odometry has been received
    bool path_loaded_;     ///<  Whether the path was loaded with success
    bool transition_pts_loaded_;  ///<  Whether the transition points have been loaded
    bool transition_;      ///<  Whether the robot is following a transition path
    std::vector<Eigen::Vector3f> transition_pts_;  ///<  Points marking start and end of transition paths

    /// \name  ROS parameters
    ///@{
    float main_freq_;   ///<  Main frequency
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
     * \brief  Loads the points delimiting the transitions
     *
     * \param file_name  Relative path of the file containing the points
     */
    void load_transition_pts(std::string file_name);

    /**
     * \brief  Finds the closest pose in a path
     *
     * \param[in] path  List of poses
     * \param[in] pose  Current pose
     * \return  Closest pose to `pose` in `path`
     */
    geometry_msgs::Pose find_closest(
      const std::vector<geometry_msgs::PoseStamped> &path,
      const geometry_msgs::Pose &pose
    );

    /**
     * \brief  Callback for the odometry
     */
     void odom_cb(const nav_msgs::Odometry msg);

};


}  // namespace mfcpp

#endif
