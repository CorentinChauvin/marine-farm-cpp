/**
 * @file
 *
 * \brief  Declaration of a node to generate hard-coded trajectories
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#ifndef TRAJ_PUBLISHER_HPP
#define TRAJ_PUBLISHER_HPP

#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <fstream>
#include <string>
#include <vector>


namespace mfcpp {

/**
 * \brief  Class to publish hard-coded trajectories
 */
class TrajPublisherNode {
  public:
    TrajPublisherNode();
    ~TrajPublisherNode();

    /**
     * \brief  Runs the node
     */
    void run_node();

  private:
    // Private members
    ros::NodeHandle nh_;  ///<  Node handler
    ros::Publisher path_pub_;   ///<  Publisher for a path
    nav_msgs::Path path_;      ///<  Path to publish
    bool path_loaded_;         ///<  Whether the path was loaded with success

    /// \name  ROS parameters
    ///@{
    float path_freq_;   ///<  Publishing frequency of the path
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

};


}  // namespace mfcpp

#endif
