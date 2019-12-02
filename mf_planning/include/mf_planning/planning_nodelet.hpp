/**
 * @file
 *
 * \brief  Declaration of a nodelet for path plannning of an underwater robot
 *         surveying a marine farm
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#ifndef PLANNING_NODELET_HPP
#define PLANNING_NODELET_HPP

#include "mf_robot_model/robot_model.hpp"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <string>
#include <csignal>

namespace mfcpp {

/**
 * \brief  Nodelet for path planning of an underwater robot surveying a marine
 *         farm
 */
class PlanningNodelet: public nodelet::Nodelet {
  public:
    PlanningNodelet();
    ~PlanningNodelet();

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
    ros::ServiceClient ray_multi_client_;  ///<  Service client for raycasting at several camera poses
    ros::Publisher lattice_pub_;  ///<  Publisher for the waypoints lattice
    ros::Publisher lattice_pose_pub_;  ///<  Publisher for the waypoints lattice poses
    tf2_ros::Buffer tf_buffer_;   ///<  Buffer for tf2
    tf2_ros::TransformListener tf_listener_;  ///<  Transform listener for tf2

    RobotModel robot_model_;  ///<  Robot model
    geometry_msgs::TransformStamped wall_robot_tf_;  ///<  Transform from wall to robot frames
    std::vector<geometry_msgs::Pose> lattice_;  ///<  Lattice of possible waypoints

    /// \name  ROS parameters
    ///@{
    float main_freq_;           ///<  Frequency of the main loop
    std::string wall_frame_;    ///<  Wall frame
    std::string robot_frame_;   ///<  Robot frame

    int nbr_int_steps_;      ///<  Initial number of model integration steps
    float max_lat_rudder_;   ///<  Maximum angle of the lateral rudder
    float max_elev_rudder_;  ///<  Maximum angle of the elevation rudder

    float plan_speed_;     ///<  Planned speed (m/s) of the robot
    float plan_horizon_;   ///<  Horizon (m) of the planning
    float lattice_res_;    ///<  Resolution (m) of the waypoints lattice
    float min_wall_dist_;  ///<  Minimum distance to the wall that can be planned
    int camera_height_;    ///<  Number of pixels of the camera along height (-1 for actual camera size)
    int camera_width_;     ///<  Number of pixels of the camera along width  (-1 for actual camera size)
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
     * \brief  Fills a lattice of possible waypoints
     *
     * Generates a lattice of possible waypoints in robot frame. These waypoints
     * will be within specified limit angles and horizon distance.
     *
     * \param [in]  max_lat_angle   Maximum lateral angle
     * \param [in]  max_elev_angle  Maximum elevation angle
     * \param [in]  horizon         Maximum distance
     * \param [in]  resolution      Spatial resolution of the lattice
     * \param [out] lattice         Lattice to fill
     */
    void generate_lattice(float max_lat_angle, float max_elev_angle,
      float horizon, float resolution, std::vector<geometry_msgs::Pose> &lattice);

    /**
     * \brief  Computes a trajectory plan
     */
    void plan_trajectory();

    /**
     * \brief  Publishes the waypoints lattice markers
     */
    void pub_lattice_markers();

    /**
     * \brief  Gets tf transforms
     *
     * \return  Whether it could retrieve a transform
     */
    bool get_tf();


};



}  // namespace mfcpp


#endif
