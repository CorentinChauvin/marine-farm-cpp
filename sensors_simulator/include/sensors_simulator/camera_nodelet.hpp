/**
 * @file
 *
 * \brief  Declaration of a nodelet simulating a camera
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "farm_simulator/rviz_visualisation.hpp"
#include "farm_simulator/Algae.h"
#include "reactphysics3d.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <csignal>
#include <string>


namespace mfcpp {


/**
 * \brief  Nodelet for a simulated camera
 */
class CameraNodelet: public nodelet::Nodelet {
  public:
    CameraNodelet();
    ~CameraNodelet();

    /**
     * \brief  Function called at beginning of nodelet execution
     */
    virtual void onInit();

  private:
    /**
     * \brief  Callback class for raycasting
     */
    class RaycastCallback: public rp3d::RaycastCallback
    {
      public:
        RaycastCallback(CameraNodelet *parent);
        virtual rp3d::decimal notifyRaycastHit(const rp3d::RaycastInfo& info);

      private:
        /// Parent CameraNodelet instance
        CameraNodelet *parent_;
    };


    // Static members
    // Note: the timers need to be static since stopped by the SIGINT callback
    static sig_atomic_t volatile b_sigint_;  ///<  Whether SIGINT signal has been received
    static ros::Timer main_timer_;   ///<  Timer callback for the main function

    // Private members
    ros::NodeHandle nh_;          ///<  Node handler (for topics and services)
    ros::NodeHandle private_nh_;  ///<  Private node handler (for parameters)
    ros::Subscriber algae_sub_;   ///<  Subscriber for the algae of the farm
    ros::Publisher rviz_pub_;     ///<  Publisher for Rviz markers
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    farm_simulator::AlgaeConstPtr last_algae_msg_;  ///<  Last algae message
    bool algae_msg_received_;  ///<  Whether an algae message has been received
    geometry_msgs::TransformStamped camera_tf_;  ///<  Transform from fixed frame to camera

    bool world_init_;   ///<  Whether the collision world has been initialised
    rp3d::CollisionWorld coll_world_;     ///<  Collision world
    rp3d::WorldSettings world_settings_;  ///<  Collision world settings
    std::vector<rp3d::CollisionBody*> coll_bodies_;  ///<  Collision bodies
    std::vector<std::unique_ptr<rp3d::BoxShape>> coll_shapes_;  ///<  Collision shapes of the bodies
    RaycastCallback raycast_cb_;

    // ROS parameters
    float camera_freq_;  ///<  Frequency of the sensor
    std::string fixed_frame_;   ///<  Frame in which the pose is expressed
    std::string camera_frame_;  ///<  Frame of the camera
    std::vector<float> fov_color_;  ///<  Color of the camera field of view Rviz marker
    float focal_length_;    ///<  Focal length of the camera
    float sensor_width_;    ///<  Width of the camera sensor
    float sensor_height_;   ///<  Height of the camera sensor
    float fov_distance_;    ///<  Maximum distance detected by the camera


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
     * \brief  Callback for the algae of the farm
     *
     * \param msg  Pointer to the algae
     */
    void algae_cb(const farm_simulator::AlgaeConstPtr msg);

    /**
     * \brief  Updates the collision world
     */
    void update_coll_world();

    /**
     * \brief  Gets tf transform from fixed frame to camera
     *
     * \return  Whether a transform has been received
     */
    bool get_camera_tf();

    /**
     * \brief  Publishes a Rviz marker for the camera field of view
     */
    void publish_rviz_fov();

    /**
     * \brief  Publishes camera output
     */
    void publish_output();

};


}  // namespace mfcpp

#endif
