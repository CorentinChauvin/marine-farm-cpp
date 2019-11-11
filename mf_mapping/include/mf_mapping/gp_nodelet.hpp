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
#include <eigen3/Eigen/Dense>
#include <csignal>
#include <cmath>
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
    ros::NodeHandle nh_;            ///<  Node handler (for topics and services)
    ros::NodeHandle private_nh_;    ///<  Private node handler (for parameters)
    ros::Subscriber camera_sub_;    ///<  Subscriber for the camera
    ros::Publisher wall_img_pub_;   ///< Publisher for an image of the wall GP
    tf2_ros::Buffer tf_buffer_;     ///<  Tf2 buffer for getting tf transforms
    tf2_ros::TransformListener tf_listener_;  ///<  Tf2 listener for getting tf transforms

    bool gp_initialised_;  ///<  Whether the Gaussian Process is initialised
    bool camera_msg_available_;  ///<  Whether a new camera message is available
    sensors_simulator::CameraOutput::ConstPtr camera_msg_;  ///<  Last camera message
    float delta_x_;  ///<  Increment (m) in the x direction
    float delta_y_;  ///<  Increment (m) in the y direction
    unsigned int size_gp_;  ///<  Total size of the Gaussian Process

    Eigen::VectorXf gp_mean_;     ///<  Mean of the Gaussian Process
    Eigen::MatrixXf gp_cov_;      ///<  Covariance of the Gaussian Process
    Eigen::MatrixXf gp_C_;  ///<  Covariance of the values of the mean of the GP
    Eigen::MatrixXf gp_C_inv_;    ///<  Inverse of gp_C_
    Eigen::VectorXf x_coord_;  ///<  X coordinate on the wall
    Eigen::VectorXf y_coord_;  ///<  Y coordinate on the wall

    // ROS parameters
    float main_freq_;     ///<  Frequency of the main loop
    std::string wall_frame_;  ///<  Name of the wall frame

    float camera_var_;    ///<  Max variance on camera measurements
    float camera_decay_;  ///<  Exponential decay rate on camera measurements

    float matern_length_;  ///<  Lengthscale of the Matern kernel
    float matern_var_;     ///<  Signal variance of the Matern kernel
    float gp_init_mean_;   ///<  Initial mean values of the Gaussian Process
    float gp_noise_var_;   ///<  Noise variance of the Gaussian Process

    float size_wall_x_;   ///<  Size (m) of the algae wall in the x direction
    float size_wall_y_;   ///<  Size (m) of the algae wall in the y direction
    int size_gp_x_;   ///<  Size of the Gaussian Process mean in the x direction
    int size_gp_y_;   ///<  Size of the Gaussian Process mean in the y direction
    int size_img_x_;  ///<  Size of the output image in the x direction
    int size_img_y_;  ///<  Size of the output image in the y direction
    int batch_size_;  ///<  Batch size for the Kalman update of the GP

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
     * \brief  Camera sensor noise model
     *
     * Computes the variance on a camera measurement. The variance depends on
     * the distance of the measured point: the greater the distance, the greater
     * the variance is.
     *
     * \param distance  Distance between the measured point and the camera origin
     */
    inline float camera_noise(float distance);

    /**
     * \brief  Matern 3/2 kernel function
     *
     * \param x1  X coordinate of the first point
     * \param y1  Y coordinate of the first point
     * \param x2  X coordinate of the second point
     * \param y2  Y coordinate of the second point
     */
    inline double matern_kernel(double x1, double y1, double x2, double y2);

    /**
     * \brief  Updates the Gaussian Process given measured data points
     *
     * \param x_meas    X coordinate of the measured data points
     * \param y_meas    Y coordinate of the measured data points
     * \param distance  Distance to the measured points
     * \param value     Value of the points at coordinates (x, y)
     */
    void update_gp(const vec_f &x_meas, const vec_f &y_meas,
      const vec_f &distance, const vec_f &values);

    /**
     * \brief  Publishes an image of the GP of an algae wall
     */
    void publish_wall_img();

};


inline float GPNodelet::camera_noise(float distance)
{
  return camera_var_ * (1 - exp(-camera_decay_ * distance));
}


inline double GPNodelet::matern_kernel(double x1, double y1, double x2, double y2)
{
  double d = sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
  double term = sqrt(3) * d / matern_length_;

  return matern_var_ * (1 + term) * exp(-term);
}



}  // namespace mfcpp

#endif
