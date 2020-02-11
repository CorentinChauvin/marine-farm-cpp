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
#include "mf_common/Float32Array.h"
#include "mf_mapping/Float32Array.h"
#include "mf_mapping/Array2D.h"
#include <nav_msgs/Path.h>
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
    ros::ServiceClient update_gp_client_;  ///<  Service client for updating the Gaussian Process
    ros::Publisher lattice_pub_;       ///<  Publisher for the waypoints lattice
    ros::Publisher lattice_pose_pub_;  ///<  Publisher for the waypoints lattice poses
    ros::Publisher path_pub_;          ///<  Publisher for the path
    ros::Subscriber gp_mean_sub_;      ///<  Subscriber for the mean of the Gaussian Process
    ros::Subscriber gp_cov_sub_;       ///<  Subscriber for the covariance of the Gaussian Process
    ros::Subscriber state_sub_;        ///<  Subscriber for the state of the robot
    tf2_ros::Buffer tf_buffer_;        ///<  Buffer for tf2
    tf2_ros::TransformListener tf_listener_;  ///<  Transform listener for tf2

    RobotModel robot_model_;        ///<  Robot model
    RobotModel::state_type state_;  ///<  State of the robot (in ocean frame)
    bool state_received_;           ///<  Whether the state of the robot has been received
    geometry_msgs::TransformStamped wall_robot_tf_;   ///<  Transform from wall to robot frames
    geometry_msgs::TransformStamped ocean_robot_tf_;  ///<  Transform from ocean to robot frames
    geometry_msgs::TransformStamped robot_ocean_tf_;  ///<  Transform from robot to ocean frames
    std::vector<geometry_msgs::Pose> lattice_;  ///<  Lattice of possible waypoints in robot frame
    int selected_vp_;  ///<  Index of selected view point in the lattice
    std::vector<float> x_hit_pt_sel_;  ///<  X coordinates of the hit points for the selected viewpoint
    std::vector<float> y_hit_pt_sel_;  ///<  Y coordinates of the hit points for the selected viewpoint
    std::vector<float> z_hit_pt_sel_;  ///<  Z coordinates of the hit points for the selected viewpoint
    std::vector<float> last_gp_mean_;              ///<  Last mean of the Gaussian Process
    std::vector<std::vector<float>> last_gp_cov_;  ///<  Last covariance of the Gaussian Process

    /// \name  General ROS parameters
    ///@{
    float main_freq_;           ///<  Frequency of the main loop
    std::string ocean_frame_;   ///<  Ocean frame
    std::string wall_frame_;    ///<  Wall frame
    std::string robot_frame_;   ///<  Robot frame
    std::string camera_frame_;  ///<  Camera frame
    ///@}

    /// \name  ROS parameters for lattice of viewpoint generation
    ///@{
    float max_lat_rudder_;   ///<  Maximum angle of the lateral rudder
    float max_elev_rudder_;  ///<  Maximum angle of the elevation rudder

    bool horiz_motion_;    ///<  Whether to allow motion in the horizontal plane
    bool vert_motion_;     ///<  Whether to allow motion in the vertical plane
    bool cart_lattice_;    ///<  Whether to create a cartesian lattice, or use motion model instead
    float plan_speed_;     ///<  Planned speed (m/s) of the robot
    float plan_horizon_;   ///<  Horizon (m) of the planning
    int lattice_size_horiz_;  ///<  Half size of the lattice in the horizontal direction
    int lattice_size_vert_;   ///<  Half size of the lattice in the vertical direction
    float wall_orientation_;   ///<  Orientation of the wall (absolute value)
    float lattice_res_;    ///<  Resolution (m) of the waypoints lattice
    std::vector<float> bnd_wall_dist_;  ///<  Bounds on the distance to the wall for waypoint selection
    std::vector<float> bnd_depth_;       ///<  Bounds on the depth (in wall frame) for waypoint selection
    ///@}

    /// \name  ROS parameters for viewpoint selection
    ///@{
    int camera_height_;  ///<  Number of pixels of the camera along height (-1 for actual camera size)
    int camera_width_;   ///<  Number of pixels of the camera along width  (-1 for actual camera size)
    float gp_weight_;    ///<  Weight attributed to the Gaussian Process values in viewpoint selection
    ///@}

    /// \name  ROS parameters for path generation
    ///@{
    bool linear_path_;     ///<  Whether to create a linear path, or a spline one
    float path_res_;       ///<  Spatial resolution (m) of the planned trajectory
    float min_wall_dist_;  ///<  Minimum distance to the wall that can be planned
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
     * \brief  Converts a 2D std vector to a custom ROS array
     *
     * \param  2D vector to convert
     * \return  Converted array
     */
    std::vector<mf_mapping::Float32Array> vector2D_to_array(
      const std::vector<std::vector<float>> &vector2D);

    /**
     * \brief  Converts a custom ROS array to a 2D std vector
     *
     * \param  The array to convert
     * \return  Converted 2D vector
     */
    std::vector<std::vector<float>> array_to_vector2D(
      const std::vector<mf_mapping::Float32Array> &array);

    /**
     * \brief  Converts a custom ROS array to a 2D std vector
     *
     * \param  The array to convert
     * \return  Converted 2D vector
     */
    std::vector<std::vector<float>> array_to_vector2D(
      const mf_mapping::Array2D &array);

    /**
     * \brief  Fills a cartesian lattice of possible waypoints
     *
     * Generates a lattice of possible waypoints in robot frame. These waypoints
     * will be aligned on a cartesian grid, within specified limit angles and
     * horizon distance.
     *
     * \param [in]  max_lat_angle   Maximum lateral angle
     * \param [in]  max_elev_angle  Maximum elevation angle
     * \param [in]  horizon         Maximum distance
     * \param [in]  resolution      Spatial resolution of the lattice
     * \param [out] lattice         Lattice to fill
     */
    void generate_cart_lattice(float max_lat_angle, float max_elev_angle,
      float horizon, float resolution, std::vector<geometry_msgs::Pose> &lattice);

    /**
     * \brief  Fills a lattice of possible waypoints based on motion model
     *
     * Generate the lattice by picking different control inputs and applying it
     * during a fixed time. The speed and horizontal angle commands will be
     * constant. The horizontal command will be interpolated to guaranty that
     * the final pose is parallel to the algae wall.
     *
     * \param[out] lattice  Lattice to fill
     */
    void generate_lattice(std::vector<geometry_msgs::Pose> &lattice);

    /**
     * \brief  Filters out waypoints that are not in given bounds
     *
     * All the waypoints will be within the bounds `bnd_wall_dist_` and `bnd_depth_`
     * with respect to the wall.
     *
     * \param lattice_in  Lattice to filter
     * \return  Filtered lattice
     */
    std::vector<geometry_msgs::Pose> filter_lattice(const std::vector<geometry_msgs::Pose> &lattice_in);

    /**
     * \brief  Computes the diagonal of the covariance for each viewpoint of the lattice
     *
     * \param[out] cov_diag  Diagonal of the GP cov for each viewpoint
     * \param[out] camera_pts_x  X coord of camera hit pts for each viewpoint
     * \param[out] camera_pts_x  Y coord of camera hit pts for each viewpoint
     * \param[out] camera_pts_x  Z coord of camera hit pts for each viewpoint
     */
    bool compute_lattice_gp(
      std::vector<std::vector<float>> &cov_diag,
      std::vector<std::vector<float>> &camera_pts_x,
      std::vector<std::vector<float>> &camera_pts_y,
      std::vector<std::vector<float>> &camera_pts_z
    );

    /**
     * \brief  Interpolates a straight line path between two poses
     *
     * \param p1          Start of the path
     * \param p2          End of the path
     * \param resolution  Spatial resolution of the path
     * \return  Generated path
     */
    nav_msgs::Path straight_line_path(const geometry_msgs::Pose &start,
      const geometry_msgs::Pose &end, float resolution);

    /**
     * \brief  Interpolates a spline path between two poses
     *
     * \param p1          Start of the path
     * \param p2          End of the path
     * \param resolution  Spatial resolution of the path
     * \param speed       Desired speed along the path
     * \return  Generated path
     */
    nav_msgs::Path spline_path(const geometry_msgs::Pose &start,
      const geometry_msgs::Pose &end, float resolution, float speed);

    /**
     * \brief  Main function to compute a trajectory plan
     */
    bool plan_trajectory();

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

    /**
     * \brief  Callback for Gaussian Process mean
     */
    void gp_mean_cb(const mf_mapping::Float32ArrayConstPtr msg);

    /**
     * \brief  Callback for Gaussian Process covariance
     */
    void gp_cov_cb(const mf_mapping::Array2DConstPtr msg);

    /**
     * \brief  Callback for the state of the robot
     */
     void state_cb(const mf_common::Float32Array msg);

    /**
     * \brief  Converts a tf transform to a pose, assuming the frames correspond
     *
     * \param transf  Transform to convert
     * \retturn  Converted pose
     */
    inline geometry_msgs::Pose tf_to_pose(const geometry_msgs::TransformStamped &transf);


};


inline geometry_msgs::Pose PlanningNodelet::tf_to_pose(
  const geometry_msgs::TransformStamped &transf
)
{
  geometry_msgs::Pose pose;
  pose.position.x = transf.transform.translation.x;
  pose.position.y = transf.transform.translation.y;
  pose.position.z = transf.transform.translation.z;
  pose.orientation.x = transf.transform.rotation.x;
  pose.orientation.y = transf.transform.rotation.y;
  pose.orientation.z = transf.transform.rotation.z;
  pose.orientation.w = transf.transform.rotation.w;

  return pose;
}



}  // namespace mfcpp


#endif
