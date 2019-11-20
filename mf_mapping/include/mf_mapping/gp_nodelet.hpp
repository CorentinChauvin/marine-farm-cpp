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

#include "mf_sensors_simulator/CameraOutput.h"
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
    ros::Publisher cov_img_pub_;    ///< Publisher for an image of the GP covariance
    tf2_ros::Buffer tf_buffer_;     ///<  Tf2 buffer for getting tf transforms
    tf2_ros::TransformListener tf_listener_;  ///<  Tf2 listener for getting tf transforms

    bool gp_initialised_;  ///<  Whether the Gaussian Process is initialised
    bool camera_msg_available_;  ///<  Whether a new camera message is available
    mf_sensors_simulator::CameraOutput::ConstPtr camera_msg_;  ///<  Last camera message
    float delta_x_;  ///<  Increment (m) in the x direction
    float delta_y_;  ///<  Increment (m) in the y direction
    unsigned int size_gp_;    ///<  Total size of the Gaussian Process
    unsigned int size_img_;   ///<  Total size of the image

    Eigen::VectorXf gp_mean_;     ///<  Mean of the Gaussian Process
    Eigen::MatrixXf gp_cov_;      ///<  Covariance of the Gaussian Process
    Eigen::MatrixXf gp_C_;  ///<  Covariance of the values of the mean of the GP
    Eigen::MatrixXf gp_C_inv_;    ///<  Inverse of gp_C_
    std::vector<unsigned int> idx_obs_;  ///<  Array of corresponding indices for observed states
    Eigen::VectorXf x_coord_;     ///<  X coordinates of the state on the wall
    Eigen::VectorXf y_coord_;     ///<  Y coordinates of the state on the wall
    std::vector<float> out_values_;  ///<  Output values of the Gaussian Process
    std::vector<bool> changed_pxl_;  ///<  Whether an output pixel has been updated

    /// \name  ROS parameters
    ///@{
    float main_freq_;         ///<  Frequency of the main loop
    std::string wall_frame_;  ///<  Name of the wall frame

    float camera_var_;     ///<  Max variance on camera measurements
    float camera_decay_;   ///<  Exponential decay rate on camera measurements

    float matern_length_;  ///<  Lengthscale of the Matern kernel
    float matern_var_;     ///<  Signal variance of the Matern kernel
    float matern_thresh_;  ///< Threshold to consider that the kernel value is 0
    float gp_init_mean_;   ///<  Initial mean values of the Gaussian Process
    float gp_noise_var_;   ///<  Noise variance of the Gaussian Process
    float gp_cov_thresh_;  ///<  Threshold to consider a value as 0 in the covariance
    float out_scale_;      ///<  Exponential scale factor for the output
                           ///<  (higher value implies sharper transition between 0 and 1)

    float size_wall_x_;   ///<  Size (m) of the algae wall in the x direction
    float size_wall_y_;   ///<  Size (m) of the algae wall in the y direction
    int size_gp_x_;   ///<  Size of the Gaussian Process mean in the x direction
    int size_gp_y_;   ///<  Size of the Gaussian Process mean in the y direction
    int size_img_x_;  ///<  Size of the output image in the x direction
    int size_img_y_;  ///<  Size of the output image in the y direction
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
     * \brief  Callback for the camera output
     *
     * \param msg  Pointer to the camera message
     */
    void camera_cb(const mf_sensors_simulator::CameraOutputConstPtr &msg);

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
     * \brief  Populates indices correspondance for reordered states
     *
     * Used to reorder the state by putting observed states at the beginning
     * and not observed states at the end.
     *
     * \param[in] size_obs   Size of the observed state
     * \param[in] size_nobs  Size of the non observed state
     * \param[in] min_x      Minimal x index of the observed states
     * \param[in] max_x      Maximal x index of the observed states
     * \param[in] min_y      Minimal y index of the observed states
     * \param[in] max_y      Maximal y index of the observed states     *
     * \param[out] idx_obs   Array of corr. indices for obs states to populate
     * \param[out] idx_nobs  Array of corr. indices for not obs states to populate
     */
    void pop_reordered_idx(
      unsigned int size_obs, unsigned int size_nobs,
      unsigned int min_x, unsigned int max_x, unsigned int min_y, unsigned int max_y,
      std::vector<unsigned int> &idx_obs, std::vector<unsigned int> &idx_nobs
    );

    /**
     * \brief  Notifies changing pixels during GP update
     *
     * \param min_x      Minimal x index of the observed states
     * \param max_x      Maximal x index of the observed states
     * \param min_y      Minimal y index of the observed states
     * \param max_y      Maximal y index of the observed states
     */
    void notif_changing_pxls(float min_x, float max_x, float min_y, float max_y);

    /**
     * \brief  Builds vectors and matrices needed during Kalman update
     *
     * \note  It assumes the following objects are already of the right size
     *
     * \param[in] idx_obs     Array of corresponding indices for obs states
     * \param[in] idx_nobs    Array of corresponding indices for non obs states
     * \param[out] mu         Reordered state
     * \param[out] mu_obs     Observed part of the state
     * \param[out] P          Reordered covariance
     * \param[out] P_obs      Observed part of the covariance
     * \param[out] B          Off diagonal block matrix in the covariance
     * \param[out] C_obs      Covariance matrix of the GP for obs states
     * \param[out] C_obs_inv  Inverse of C_obs
     * \param[out] x_coord  X coordinates of the reordered state
     * \param[out] y_coord  Y coordinates of the reordered state
     */
    void build_Kalman_objects(
      const std::vector<unsigned int> &idx_obs,
      const std::vector<unsigned int> &idx_nobs,
      Eigen::VectorXf &mu, Eigen::VectorXf &mu_obs,
      Eigen::MatrixXf &P, Eigen::MatrixXf &P_obs, Eigen::MatrixXf &B,
      Eigen::MatrixXf &C_obs, Eigen::MatrixXf &C_obs_inv,
      Eigen::VectorXf &x_coord, Eigen::VectorXf &y_coord
    );

    /**
     * \brief  Builds vectors and matrices needed during GP evaluation
     *
     * \note  It assumes the following objects are already of the right size
     *
     * \param[in] idx_obs     Array of corresponding indices for obs states
     * \param[in] idx_nobs    Array of corresponding indices for non obs states
     * \param[out] mu_nobs    Not observed part of the state
     * \param[out] C_nobs     Covariance matrix of the GP for not obs states
     * \param[out] E          Off diagonal block matrix in gp_C_
     */
    void build_eval_objects(
      const std::vector<unsigned int> &idx_obs,
      const std::vector<unsigned int> &idx_nobs,
      Eigen::VectorXf &mu_nobs, Eigen::MatrixXf &C_nobs, Eigen::MatrixXf &E
    );

    /**
     * \brief  Fills GP mean and covariance from reordered objects
     *
     * \param idx_obs   Array of corresponding indices for obs states
     * \param idx_nobs  Array of corresponding indices for non obs states
     * \param mu        Reordered state
     * \param P         Reordered covariance
     */
    void update_reordered_gp(
      const std::vector<unsigned int> &idx_obs,
      const std::vector<unsigned int> &idx_nobs,
      Eigen::VectorXf &mu, const Eigen::MatrixXf &P
    );

    /**
     * \brief  Updates the Gaussian Process given measured data points
     *
     * \param x_meas     X coordinate of the measured data points
     * \param y_meas     Y coordinate of the measured data points
     * \param z_meas     Z coordinate of the measured data points
     * \param distances  Distances to the measured points
     * \param value     Value of the points at coordinates (x, y)
     */
    void update_gp(const vec_f &x_meas, const vec_f &y_meas,
      const vec_f &z, const vec_f &distances, const vec_f &values);

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
