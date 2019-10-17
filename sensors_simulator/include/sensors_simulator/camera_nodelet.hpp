/**
 * @file
 *
 * \brief  Declaration of a nodelet simulating a camera
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "farm_simulator/Algae.h"
#include "reactphysics3d.h"
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <csignal>


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
    // Static members
    // Note: the timers need to be static since stopped by the SIGINT callback
    static sig_atomic_t volatile b_sigint_;  ///<  Whether SIGINT signal has been received
    static ros::Timer main_timer_;   ///<  Timer callback for the main function

    // Private members
    ros::NodeHandle nh_;          ///<  Node handler (for topics and services)
    ros::NodeHandle private_nh_;  ///<  Private node handler (for parameters)
    ros::Subscriber algae_sub_;   ///<  Subscriber for the algae of the farm
    farm_simulator::AlgaeConstPtr last_algae_msg_;  ///<  Last algae message
    bool algae_msg_received_;  ///<  Whether an algae message has been received
    rp3d::CollisionWorld coll_world_;     ///<  Collision world
    rp3d::WorldSettings world_settings_;  ///<  Collision world settings

    // ROS parameters
    float camera_freq_;  ///<  Frequency of the sensor


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


};


}  // namespace mfcpp

#endif
