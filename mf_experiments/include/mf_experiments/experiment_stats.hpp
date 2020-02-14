/**
 * @file
 *
 * \brief  Declaration of a node to measure statistics about experiments.
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#ifndef EXPERIMENT_STATS_HPP
#define EXPERIMENT_STATS_HPP

#include "mf_common/EulerPose.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <fstream>
#include <string>
#include <vector>


namespace mfcpp {

/**
 * \brief  Class to measure statistics about experiments
 *
 * Writes reference tracking error in a file, and publish diverse errors as ROS
 * topics.
 */
class ExperimentStatsNode {
  public:
    ExperimentStatsNode();
    ~ExperimentStatsNode();

    /**
     * \brief  Runs the node
     */
    void run_node();

  private:
    // Private members
    ros::NodeHandle nh_;  ///<  Node handler
    ros::Subscriber odom_sub_;  ///<  Subscriber for robot odometry
    ros::Subscriber path_sub_;  ///<  Subscriber for the reference path
    ros::Publisher ref_pub_;    ///<  Publisher for the reference pose
    ros::Publisher error_pub_;  ///<  Publisher for the tracking error

    nav_msgs::Odometry odom_;  ///<  Last odometry message received
    bool odom_received_;       ///<  Whether odometry has been received
    nav_msgs::Path path_;      ///<  Reference path to follow
    bool path_received_;       ///<  Whether the reference path has been received
    std::ofstream out_file_;   ///<  Output CSV file containg data of the experiment
    double start_time_;        ///<  Start time of the test


    /// \name  ROS parameters
    ///@{
    float main_freq_;   ///<  Frequency of the main loop
    ///@}


    /**
     * \brief  Initialises the node and its parameters
     */
    void init_node();

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
     * \brief  Writes the output statistics in the out file
     *
     * \param pose        Current pose
     * \param reference   Reference pose
     * \param error       Error between current pose and reference
     */
    void write_output(
      const geometry_msgs::Pose &pose,
      const geometry_msgs::Pose &reference,
      const mf_common::EulerPose &error
    );

    /**
     * \brief  Callback for odometry
     */
    void odom_cb(const nav_msgs::Odometry msg);

    /**
     * \brief  Callback for the reference path
     */
    void path_cb(const nav_msgs::Path msg);

};


}  // namespace mfcpp

#endif
