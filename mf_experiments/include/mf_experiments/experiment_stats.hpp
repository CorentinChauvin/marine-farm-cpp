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
#include "mf_common/Array2D.h"
#include "mf_common/Float32Array.h"
#include "mf_farm_simulator/Algae.h"
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
 * Measures different statistics and publishes it to ROS and to a text file.
 * Statistics measured:
 *  - control tracking error
 *  - trace of the Gaussian Process covariance
 *  - information (weighted sum of the diagonal coefficients of the covariance of the GP)
 *  - mapping error
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
    ros::Subscriber odom_sub_;     ///<  Subscriber for robot odometry
    ros::Subscriber path_sub_;     ///<  Subscriber for the reference path
    ros::Subscriber gp_mean_sub_;  ///<  Subscriber for mean of the GP
    ros::Subscriber gp_cov_sub_;   ///<  Subscriber for covariance of the GP
    ros::Subscriber gp_eval_sub_;  ///<  Subscriber for the evaluated GP
    ros::Subscriber algae_sub_;    ///<  Subscriber for the farm algae
    ros::Publisher ref_pub_;       ///<  Publisher for the reference pose
    ros::Publisher error_pub_;     ///<  Publisher for the tracking error
    ros::Publisher diff_img_pub_;  ///<  Publisher for the image of the difference between the mapped GP and the algae heatmap

    nav_msgs::Odometry odom_;     ///<  Last odometry message received
    nav_msgs::Path path_;         ///<  Reference path to follow
    std::vector<float> gp_mean_;  ///<  Mean of the Gaussian Process
    std::vector<float> gp_cov_;   ///<  Diagonal of the covariance of the Gaussian Proces
    std::vector<float> init_gp_cov_;  ///<  Diagonal of the initial covariance of the GP
    std::vector<std::vector<float>> gp_eval_;  ///<  Evaluated GP
    std::vector<std::vector<std::vector<float>>> algae_heatmaps_;  ///<  Disease heatmaps of the algae
    bool odom_received_;      ///<  Whether odometry has been received
    bool path_received_;      ///<  Whether the reference path has been received
    bool gp_mean_received_;   ///<  Whether the mean of the GP has been received
    bool gp_cov_received_;    ///<  Whether the covariance of the GP has been received
    bool gp_eval_received_;   ///<  Whether the evaluated GP has been received
    bool algae_received_;     ///<  Whether the farm algae have been received

    double start_time_;       ///<  Start time of the test
    std::ofstream out_file_;  ///<  Output CSV file containg data of the experiment

    /// \name  ROS parameters
    ///@{
    float main_freq_;   ///<  Frequency of the main loop
    float gp_weight_;   ///<  Weight attributed to the Gaussian Process values in viewpoint selection
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
     * \brief  Computes the difference between two arrays
     *
     * Computes the 2-norm of the difference between the two arrays, and
     * publishes an image corresponding to the difference.
     *
     * The arrays don't need to be of the same size (but of same proportions).
     * The values will be scaled out by their maximum value.
     *
     * \param array1  First array
     * \param array2  Second array
     * \return  2-norm of the difference of the two arrays
     */
    float compute_diff(
      const std::vector<std::vector<float>> &array1,
      const std::vector<std::vector<float>> &array2
    );

    /**
     * \brief  Writes the output statistics in the out file
     *
     * \param pose          Current pose
     * \param reference     Reference pose
     * \param error         Error between current pose and reference
     * \param gp_cov_trace  Trace of the covariance of the Gaussian Process
     * \param information   Information (weighted sum of the diagonal coefficients of the covariance of the GP)
     * \param gp_diff_norm  Norm of the difference between the evaluated GP and the algae disease heatmaps
     */
    void write_output(
      const geometry_msgs::Pose &pose,
      const geometry_msgs::Pose &reference,
      const mf_common::EulerPose &error,
      float gp_cov_trace,
      float information,
      float gp_diff_norm
    );

    /**
     * \brief  Callback for odometry
     */
    void odom_cb(const nav_msgs::Odometry msg);

    /**
     * \brief  Callback for the reference path
     */
    void path_cb(const nav_msgs::Path msg);

    /**
     * \brief  Callback for the mean of the Gaussian Process
     */
    void gp_mean_cb(const mf_common::Float32Array msg);

    /**
     * \brief  Callback for the covariance of the Gaussian Process
     */
    void gp_cov_cb(const mf_common::Array2D msg);

    /**
     * \brief  Callback for the evaluated GP
     */
    void gp_eval_cb(const mf_common::Array2D msg);

    /**
     * \brief  Callback for farm algae
     */
    void algae_cb(const mf_farm_simulator::Algae msg);


};


}  // namespace mfcpp

#endif
