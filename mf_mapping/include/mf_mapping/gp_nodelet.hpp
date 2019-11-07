/**
 * @file
 *
 * \brief  Declaration of a nodelet for Gaussian Process mapping of an
 *         algae wall
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#ifndef GP_NODELET_HPP
#define GP_NODELET_HPP

#include "sensors_simulator/CameraOutput.h"
#include <tf2_ros/transform_listener.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <csignal>
#include <string>
#include <vector>


namespace mfcpp {


/**
 * \brief  Nodelet for a Gaussian Process mapping an algae wall
 */
class GPNodelet: public nodelet::Nodelet {
  public:
    GPNodelet();
    ~GPNodelet();

    /**
     * \brief  Function called at beginning of nodelet execution
     */
    virtual void onInit();

  private:
    // Typedefs
    typedef std::vector<float> vec_f;

    // Note: the timers need to be static since stopped by the SIGINT callback
    static sig_atomic_t volatile b_sigint_;  ///<  Whether SIGINT signal has been received
    static ros::Timer main_timer_;   ///<  Timer callback for the main function

    // Private members
    ros::NodeHandle nh_;          ///<  Node handler (for topics and services)
    ros::NodeHandle private_nh_;  ///<  Private node handler (for parameters)
    ros::Subscriber camera_sub_;  ///<  Subscriber for the camera
    tf2_ros::Buffer tf_buffer_;   ///<  Tf2 buffer for getting tf transforms
    tf2_ros::TransformListener tf_listener_;  ///<  Tf2 listener for getting tf transforms

    bool gp_initialised_;  ///<  Whether the Gaussian Process is initialised
    bool camera_msg_available_;  ///<  Whether a new camera message is available
    sensors_simulator::CameraOutput::ConstPtr camera_msg_;  ///<  Last camera message


    // ROS parameters
    float main_freq_;  ///<  Frequency of the main loop
    std::string wall_frame_;  ///<  Name of the wall frame

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
     * \brief  Callback for the camera output
     *
     * \param msg  Pointer to the camera message
     */
    void camera_cb(const sensors_simulator::CameraOutputConstPtr &msg);

    /**
     * \brief  Transforms points from camera frame to wall frame
     *
     * \param x_in      X coordinate of the input point
     * \param y_in      Y coordinate of the input point
     * \param z_in      Z coordinate of the input point
     * \param x_out     X coordinate of the transformed point
     * \param y_out     Y coordinate of the transformed point
     * \param z_out     Z coordinate of the transformed point
     * \param frame_in  Frame of the input point
     *
     * \return  Whether the points could be transformed
     */
    bool transform_points(const vec_f &x_in, const vec_f &y_in, const vec_f &z_in,
      vec_f &x_out, vec_f &y_out, vec_f &z_out,
      std::string frame_in, std::string frame_out);

    /**
     * \brief  Initialises the Gaussian Process
     */
    void init_gp();

    /**
     * \brief  Updates the Gaussian Process given measured data points
     *
     * \param x      X coordinate of the measured data points
     * \param y      Y coordinate of the measured data points
     * \param z      Z coordinate of the measured data points
     * \param value  Value of the points at coordinates (x, y, z)
     */
    void update_gp(const vec_f &x, const vec_f &y, const vec_f &z,
      const vec_f &values);


};



}  // namespace mfcpp

#endif
