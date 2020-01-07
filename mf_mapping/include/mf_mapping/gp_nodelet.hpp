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

#include "mf_mapping/UpdateGP.h"
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
    /**
     * \brief  2D rectangular area
     *
     * \param min_x      Minimal x coordinate
     * \param max_x      Maximal x coordinate
     * \param min_y      Minimal y coordinate
     * \param max_y      Maximal y coordinate
     */
    struct RectArea {
      float min_x;
      float max_x;
      float min_y;
      float max_y;
    };

    // Typedefs
    typedef std::vector<float> vec_f;

    // Static members
    // Note: the timers need to be static since stopped by the SIGINT callback
    static sig_atomic_t volatile b_sigint_;  ///<  Whether SIGINT signal has been received
    static ros::Timer main_timer_;   ///<  Timer callback for the main function

    // Private members
    ros::NodeHandle nh_;            ///<  Node handler (for topics and services)
    ros::NodeHandle private_nh_;    ///<  Private node handler (for parameters)
    ros::Subscriber camera_sub_;    ///<  Subscriber for the camera
    ros::Publisher wall_img_pub_;   ///<  Publisher for an image of the wall GP
    ros::Publisher cov_img_pub_;    ///<  Publisher for an image of the GP covariance
    ros::Publisher gp_mean_pub_;    ///<  Publisher for the GP mean
    ros::Publisher gp_cov_pub_;     ///<  Publisher for the GP covariance
    ros::ServiceServer update_gp_serv_;  ///<  Service server for updating the GP
    tf2_ros::Buffer tf_buffer_;     ///<  Tf2 buffer for getting tf transforms
    tf2_ros::TransformListener tf_listener_;  ///<  Tf2 listener for getting tf transforms

    bool gp_initialised_;  ///<  Whether the Gaussian Process is initialised
    bool camera_msg_available_;  ///<  Whether a new camera message is available
    bool tf_got_;   ///<  Whether transforms have been retrieved
    mf_sensors_simulator::CameraOutput::ConstPtr camera_msg_;  ///<  Last camera message
    float delta_x_;  ///<  Increment (m) in the x direction
    float delta_y_;  ///<  Increment (m) in the y direction
    unsigned int size_gp_;    ///<  Total size of the Gaussian Process
    unsigned int size_img_;   ///<  Total size of the image
    geometry_msgs::TransformStamped wall_camera_tf_;  // Transform from wall to camera frames
    float radius_obs_;  ///<  Radius of influence of an observation on the GP

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
    float main_freq_;           ///<  Frequency of the main loop
    std::string wall_frame_;    ///<  Name of the wall frame
    std::string camera_frame_;  ///<  Name of the camera frame

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
     * \brief  Gets necessary tf transforms
     *
     * \return  Whether the transforms could be retrieved
     */
    bool get_tf();

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
     * \brief  Service to update the Gaussian Process for several sets of measurements
     */
    bool update_gp_cb(mf_mapping::UpdateGP::Request &req,
      mf_mapping::UpdateGP::Response &res);

    /**
     * \brief  Transforms points from one frame to another
     *
     * \param[in]  x_in       X coordinate of the input points
     * \param[in]  y_in       Y coordinate of the input points
     * \param[in]  z_in       Z coordinate of the input points
     * \param[out] x_out      X coordinate of the transformed points
     * \param[out] y_out      Y coordinate of the transformed points
     * \param[out] z_out      Z coordinate of the transformed points
     * \param[in]  frame_in   Frame of the input points
     * \param[in]  frame_out  Frame of the output points
     *
     * \return  Whether the points could be transformed
     */
    bool transform_points(const vec_f &x_in, const vec_f &y_in, const vec_f &z_in,
      vec_f &x_out, vec_f &y_out, vec_f &z_out,
      std::string frame_in, std::string frame_out);

    /**
     * \brief  Transforms points from one frame to another
     *
     * \param[in]  x_in       X coordinate of the input points
     * \param[in]  y_in       Y coordinate of the input points
     * \param[in]  z_in       Z coordinate of the input points
     * \param[out] x_out      X coordinate of the transformed points
     * \param[out] y_out      Y coordinate of the transformed points
     * \param[out] z_out      Z coordinate of the transformed points
     * \param[in]  transform  Transform to apply
     *
     * \return  Whether the points could be transformed
     */
    bool transform_points(const vec_f &x_in, const vec_f &y_in, const vec_f &z_in,
      vec_f &x_out, vec_f &y_out, vec_f &z_out,
      const geometry_msgs::TransformStamped &transform);

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
    inline float camera_noise(float distance) const;

    /**
     * \brief  Matern 3/2 kernel function
     *
     * \param x1  X coordinate of the first point
     * \param y1  Y coordinate of the first point
     * \param x2  X coordinate of the second point
     * \param y2  Y coordinate of the second point
     */
    inline double matern_kernel(double x1, double y1, double x2, double y2) const;

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
     * \param[in] max_y      Maximal y index of the observed states
     * \param[out] idx_obs   Array of corr. indices for obs states to populate
     * \param[out] idx_nobs  Array of corr. indices for not obs states to populate
     */
    void pop_reordered_idx(
      unsigned int size_obs, unsigned int size_nobs,
      unsigned int min_x, unsigned int max_x, unsigned int min_y, unsigned int max_y,
      std::vector<unsigned int> &idx_obs, std::vector<unsigned int> &idx_nobs
    ) const;

    /**
     * \brief  Notifies changing pixels during GP update
     *
     * \param coord  Coordinates of the rectangular area of the observed state
     */
    void notif_changing_pxls(const RectArea &coord);

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
    ) const;

    /**
     * \brief  Fills GP mean and covariance from reordered objects
     *
     * \param[in]  idx_obs   Array of corresponding indices for obs states
     * \param[in]  idx_nobs  Array of corresponding indices for non obs states
     * \param[in]  mu        Reordered state
     * \param[in]  P         Reordered covariance
     * \param[out] gp_mean   GP mean to fill
     * \param[out] gp_cov    GP covariance to fill
     */
    void update_reordered_gp(
      const std::vector<unsigned int> &idx_obs,
      const std::vector<unsigned int> &idx_nobs,
      const Eigen::VectorXf &mu, const Eigen::MatrixXf &P,
      Eigen::VectorXf &gp_mean, Eigen::MatrixXf &gp_cov
    ) const;

    /**
     * \brief  Updates the Gaussian Process given measured data points
     *
     * \param[in]      x_meas       X coordinate of the measured data points (in wall frame=
     * \param[in]      y_meas       Y coordinate of the measured data points (in wall frame=
     * \param[in]      z_meas       Z coordinate of the measured data points (in wall frame=
     * \param[in]      distances    Distances between the camera origin and the measured points
     * \param[in]      values       Measured value of the points at coordinates (x, y)
     * \param[in, out] gp_mean      Mean of the Gaussian Process
     * \param[in, out] gp_cov       Covariance of the Gaussian Process
     * \param[out]     idx_obs      Array of corresponding indices for obs states
     * \param[out]     obs_coord    Coordinates of the rectangular area of the observed state
     * \param[in]      update_mean  Whether to update the mean
     */
    void update_gp(
      const vec_f &x_meas, const vec_f &y_meas, const vec_f &z_meas,
      const vec_f &distances, const vec_f &values,
      Eigen::VectorXf &gp_mean, Eigen::MatrixXf &gp_cov,
      std::vector<unsigned int> &idx_obs,
      RectArea &obs_coord,
      bool update_mean = true
    ) const;

    /**
     * \brief  Prepares objects needed for Gaussian Process evaluation
     *
     * \param[in]  idx_obs  Array of corresponding indices for obs states
     * \param[in]  gp_mean  Mean of the Gaussian Process
     * \param[out] x_obs    X coordinates of the observed state
     * \param[out] y_obs    Y coordinates of the observed state
     * \param[out] W        Vector needed for evaluation (\f$ W = (C^{-1})_{obs} \mu_{obs} \f$)
     */
    void prepare_eval(
      const std::vector<unsigned int> &idx_obs,
      const Eigen::VectorXf &gp_mean,
      Eigen::VectorXf &x_obs, Eigen::VectorXf &y_obs,
      Eigen::VectorXf &W
    ) const;

    /**
     * \brief  Evaluates the Gaussian Process at a given point
     *
     * \param x      X coordinate of the evaluated point
     * \param y      Y coordinate of the evaluated point
     * \param x_obs  X coordinates of the observed state
     * \param x_obs  Y coordinates of the observed state
     * \param W      Vector needed for evaluation (\f$ W = (C^{-1})_{obs} \mu_{obs} \f$)
     * \return  Value of the evaluated Gaussian Process
     */
    float eval_gp(
      float x, float y,
      const Eigen::VectorXf &x_obs, const Eigen::VectorXf &y_obs,
      const Eigen::VectorXf &W
    ) const;

    /**
     * \brief  Publishes mean and covariance of the Gaussian Process
     */
    void publish_gp_state();

    /**
     * \brief  Publishes an image of the evaluated GP and its covariance
     */
    void publish_wall_img();

    /**
     * \brief  Evaluates the Lambert W (0 branch) function
     *
     * The Lambert W function solves the equation w*exp(w) = z with w = W0(z)
     * when z >= 0. Uses Newton's method as indicated at
     * https://en.wikipedia.org/wiki/Lambert_W_function#Numerical_evaluation
     *
     * \note  This function is better implemented in Boost in versions >=1.69,
     *        which is not supported by ROS 1 (as of January 2020)
     *
     * \param z          Positive real number
     * \param precision  Desired precision for the result
     * \param init_w     Initial point of Newton's evaluation
     * \return  The Lambert W function evaluated at z if z >= 0, 0 otherwise.
     */
    double lambert_w0(double z, double precision=0.01, double init_w=1.0) const;

    /**
     * \brief  Evaluates the Lambert W (-1 branch) function
     *
     * The Lambert W function solves the equation w*exp(w) = z with w = W0(z)
     * when -1/e <= z < 0. Uses Newton's method as indicated at
     * https://en.wikipedia.org/wiki/Lambert_W_function#Numerical_evaluation
     *
     * \note  This function is better implemented in Boost in versions >=1.69,
     *        which is not supported by ROS 1 (as of January 2020)
     *
     * \param z          Real number in [-1/e, 0)
     * \param precision  Desired precision for the result
     * \param init_w     Initial point of Newton's evaluation
     * \return  The Lambert W function evaluated at z if z is in the right bound,
     *          0 otherwise.
     */
    double lambert_wm1(double z, double precision=0.01, double init_w=-2.0) const;

    /**
     * \brief  Applies Netwton's method to evaluate Lambert W function
     *
     * The initial point should be < -1.0 for branch -1 and > 0.0 for branch 0.
     *
     * \warning  Does not care of bounds, so to use with care. Prefer using
     *    `GPNodelet::lambert_w0` and `GPNodelet::lambert_wm1`.
     *
     * \param z          Real number in [-1/e, +inf)
     * \param precision  Desired precision for the result
     * \param init_w     Initial point of Newton's evaluation
     * \return  The Lambert W function evaluated at z.
     */
    double lambert_w(double z, double precision, double init_w) const;

};


inline float GPNodelet::camera_noise(float distance) const
{
  return camera_var_ * (1 - exp(-camera_decay_ * distance));
}


inline double GPNodelet::matern_kernel(double x1, double y1, double x2, double y2) const
{
  double d = sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
  double term = sqrt(3) * d / matern_length_;

  return matern_var_ * (1 + term) * exp(-term);
}



}  // namespace mfcpp

#endif
