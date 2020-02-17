/**
 * @file
 *
 * \brief  Definition of a node to generate a fake trajectory and evaluate the
 *         performance of the control.
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#include "experiment_stats.hpp"
#include "mf_common/common.hpp"
#include "mf_common/spline.hpp"
#include "mf_common/EulerPose.h"
#include "mf_common/Array2D.h"
#include "mf_common/Float32Array.h"
#include "mf_farm_simulator/Algae.h"
#include "mf_farm_simulator/Alga.h"
#include "mf_mapping/LoadGP.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


using std::cout;
using std::endl;
using std::string;
using std::vector;


namespace mfcpp {


ExperimentStatsNode::ExperimentStatsNode():
  nh_("~")
{
 init_node();
}


ExperimentStatsNode::~ExperimentStatsNode()
{

}


void ExperimentStatsNode::init_node()
{
  // ROS parameters
  string out_file_name;  // path of the output file for the results of the test

  nh_.param<float>("main_freq", main_freq_, 1.0);
  nh_.param<string>("out_file_name", out_file_name, "/tmp/control_out.txt");
  nh_.param<float>("gp_weight", gp_weight_, 1.0);
  nh_.param<bool>("save_gp", save_gp_, false);
  nh_.param<bool>("load_gp", load_gp_, false);
  nh_.param<string>("gp_file_name", gp_file_name_, "/tmp/gp_mean.txt");

  // Other variables
  odom_received_ = false;
  path_received_ = false;
  gp_cov_received_ = false;
  gp_eval_received_ = false;
  algae_received_ = false;
  start_time_ = ros::Time::now().toSec();

  out_file_.open(out_file_name);
  if (out_file_.is_open())
    out_file_ << "t, x, y, z, x_ref, y_ref, z_ref, pos_err, orient_err, gp_cov_trace, gp_diff_norm" << endl;
  else
    ROS_WARN("[experiment_stats] Couldn't open file '%s' in write mode", out_file_name.c_str());

  // ROS subscribers
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1, &ExperimentStatsNode::odom_cb, this);
  path_sub_ = nh_.subscribe<nav_msgs::Path>("path", 1, &ExperimentStatsNode::path_cb, this);
  gp_mean_sub_ = nh_.subscribe<mf_common::Float32Array>("gp_mean", 1, &ExperimentStatsNode::gp_mean_cb, this);
  gp_cov_sub_ = nh_.subscribe<mf_common::Array2D>("gp_cov", 1, &ExperimentStatsNode::gp_cov_cb, this);
  gp_eval_sub_ = nh_.subscribe<mf_common::Array2D>("gp_eval", 1, &ExperimentStatsNode::gp_eval_cb, this);
  algae_sub_ = nh_.subscribe<mf_farm_simulator::Algae>("algae", 1, &ExperimentStatsNode::algae_cb, this);

  // ROS publishers
  ref_pub_ = nh_.advertise<geometry_msgs::Pose>("reference", 0);
  error_pub_ = nh_.advertise<mf_common::EulerPose>("error", 0);
  diff_img_pub_ = nh_.advertise<sensor_msgs::Image>("diff_gp_img", 0);

  // ROS services
  load_gp_client_ = nh_.serviceClient<mf_mapping::LoadGP>("load_gp_srv");

}


void ExperimentStatsNode::run_node()
{
  ros::Rate loop_rate(main_freq_);
  ros::Time t_path = ros::Time::now();  // last time path has been published

  while (ros::ok()) {
    ros::spinOnce();

    // Attempt to load the GP if requested
    if (load_gp_ && load_gp_client_.exists()) {
      vector<float> gp_mean;
      bool gp_loaded = load_gp(gp_file_name_, gp_mean);

      if (gp_loaded) {
        mf_mapping::LoadGP srv;
        srv.request.mean = gp_mean;

        bool srv_called = load_gp_client_.call(srv);

        if (srv_called && srv.response.gp_loaded) {
          load_gp_ = false;
          ROS_INFO("[experiment_stats] GP loaded");
        } else if (!srv_called || (srv_called && !srv.response.gp_not_yet_init)){
          ROS_WARN("[experiment_stats] Could not load GP");
        }
      }
    }

    // Compute statistics
    if (path_received_ && odom_received_ && gp_cov_received_ && gp_mean_received_
      && gp_eval_received_ && algae_received_) {
      // Compute tracking error
      geometry_msgs::Pose reference_pose = find_closest(path_.poses, odom_.pose.pose);
      mf_common::EulerPose error;
      diff_pose(reference_pose, odom_.pose.pose, error);

      // Compute trace of GP covariance and information
      float gp_cov_trace = 0.0;
      float information = 0.0;
      int size_gp = gp_cov_.size();

      for (int k = 0; k < size_gp; k++) {
        gp_cov_trace += gp_cov_[k];

        float weight = 1/(1 + exp(-gp_weight_ * (gp_mean_[k] - 0.5)));
        information += weight * (init_gp_cov_[k] - gp_cov_[k]);
      }

      // Compare evaluated GP and algae disease heatmaps
      float gp_diff_norm = compute_diff(gp_eval_, algae_heatmaps_[0]);

      // Write statistics in the output file
      write_output(odom_.pose.pose, reference_pose, error, gp_cov_trace, information, gp_diff_norm);

      // Publish statistics
      ref_pub_.publish(reference_pose);
      error_pub_.publish(error);
    }

    loop_rate.sleep();
  }

  // Save the GP if requested
  if (save_gp_ && gp_mean_received_)
    save_gp();

  // Close the output CSV file
  if (out_file_.is_open())
    out_file_.close();
}


geometry_msgs::Pose ExperimentStatsNode::find_closest(
  const std::vector<geometry_msgs::PoseStamped> &path,
  const geometry_msgs::Pose &pose)
{
  if (path.size() == 0)
    return geometry_msgs::Pose();

  float min_dist = distance2(path[0].pose.position, pose.position);
  int best_k = 0;

  for (int k = 1; k < path.size(); k++) {
    float d = distance2(path[k].pose.position, pose.position);

    if (d < min_dist) {
      min_dist = d;
      best_k = k;
    }
  }

  return path[best_k].pose;
}


float ExperimentStatsNode::compute_diff(
  const std::vector<std::vector<float>> &array1,
  const std::vector<std::vector<float>> &array2)
{
  const vector<vector<float>> *small_array;
  const vector<vector<float>> *big_array;

  // Find the smallest array
  if (array1.size() < array2.size()) {
    small_array = &array1;
    big_array = &array2;
  } else {
    small_array = &array2;
    big_array = &array1;
  }

  float size_x[2] = {(float)(*small_array).size(), (float)(*big_array).size()};
  float size_y[2] = {(float)(*small_array)[0].size(), (float)(*big_array)[0].size()};

  // Find the maximum of the two arrays for scaling
  typedef const vector<vector<float>>* Array2DPtr;
  Array2DPtr arrays[2] = {small_array, big_array};
  float maximums[2] = {0, 0};

  for (int k = 0; k < 2; k++) {
    for (int x = 0; x < size_x[k]; x++) {
      for (int y = 0; y < size_y[k]; y++) {
        if ((*(arrays[k]))[x][y] > maximums[k])
          maximums[k] = (*(arrays[k]))[x][y];
      }
    }
  }

  // Prepare the difference image
  sensor_msgs::Image diff_img;
  diff_img.header.stamp = ros::Time::now();
  diff_img.height = size_x[0];
  diff_img.width = size_y[0];
  diff_img.encoding = sensor_msgs::image_encodings::MONO8;
  diff_img.is_bigendian = false;
  diff_img.step = size_y[0];
  diff_img.data.resize(size_x[0]*size_y[0]);

  // Compute the 2-norm of the two arrays
  float norm = 0.0;

  for (int k = 0; k < size_x[0]; k++) {
    int x = k * float(size_x[1] / size_x[0]);

    for (int l = 0; l < size_y[0]; l++) {
      int y = l * float(size_y[1] / size_y[0]);
      float small_val = (*small_array)[k][l];
      float big_val = (*big_array)[x][y];

      norm += pow(small_val/maximums[0] - big_val/maximums[1], 2);
      diff_img.data[k*size_y[0] + l] = fabs(small_val/maximums[0] - big_val/maximums[1])*255;
    }
  }

  diff_img_pub_.publish(diff_img);
  return sqrt(norm) / (size_x[0]*size_y[0]);
}


void ExperimentStatsNode::write_output(
  const geometry_msgs::Pose &pose,
  const geometry_msgs::Pose &reference,
  const mf_common::EulerPose &error,
  float gp_cov_trace,
  float information,
  float gp_diff_norm)
{
  if (!out_file_.is_open())
    return;

  float error_pos = sqrt(error.x*error.x + error.y*error.y + error.z*error.z);
  float error_orient = sqrt(error.pitch*error.pitch + error.yaw*error.yaw);  // the roll is not controlled
  double t = ros::Time::now().toSec() - start_time_;

  out_file_ << t << ", "
            << pose.position.x << ", "
            << pose.position.y << ", "
            << pose.position.z << ", "
            << reference.position.x << ", "
            << reference.position.y << ", "
            << reference.position.z << ", "
            << error_pos << ", "
            << error_orient << ", "
            << gp_cov_trace << ", "
            << information << ", "
            << gp_diff_norm << endl;
}


bool ExperimentStatsNode::load_gp(
  std::string file_name,
  std::vector<float> &gp_mean)
{
  // Open file to write to
  std::ifstream file(gp_file_name_);

  if (!file.is_open()) {
    ROS_WARN("[experiment_stats] Couldn't open file '%s' in write mode", gp_file_name_.c_str());
    return false;
  }

  // Read the file
  ROS_INFO("[experiment_stats] Loading GP mean from '%s'\n", gp_file_name_.c_str());
  gp_mean.resize(0);

  for (string line; std::getline(file, line); ) {
    if (!line.empty()) {
      gp_mean.emplace_back(stof(line));
    }
  }

  return true;
}


void ExperimentStatsNode::save_gp()
{
  // Open file to write to
  std::ofstream file(gp_file_name_);

  if (!file.is_open()) {
    ROS_WARN("[experiment_stats] Couldn't open file '%s' in write mode", gp_file_name_.c_str());
    return;
  }

  // Write the GP mean in the file
  printf("[experiment_stats] Saving GP to '%s'\n", gp_file_name_.c_str());

  for (int k = 0; k < gp_mean_.size(); k++) {
    file << gp_mean_[k] << endl;
  }

  // Close the file
  file.close();
}


void ExperimentStatsNode::odom_cb(const nav_msgs::Odometry msg)
{
  odom_ = msg;
  odom_received_ = true;
}


void ExperimentStatsNode::path_cb(const nav_msgs::Path msg)
{
  path_ = msg;
  path_received_ = true;
}


void ExperimentStatsNode::gp_mean_cb(const mf_common::Float32Array msg)
{
  gp_mean_ = msg.data;
  gp_mean_received_ = true;
}


void ExperimentStatsNode::gp_cov_cb(const mf_common::Array2D msg)
{
  int n = msg.data.size();
  gp_cov_.resize(n);

  for (int k = 0; k < n; k++)
    gp_cov_[k] = msg.data[k].data[k];

  if (!gp_cov_received_) {
    init_gp_cov_ = gp_cov_;
    gp_cov_received_ = true;
  }
}


void ExperimentStatsNode::gp_eval_cb(const mf_common::Array2D msg)
{
  gp_eval_.resize(msg.data.size());

  for (int k = 0; k < msg.data.size(); k++) {
    gp_eval_[k].resize(msg.data[k].data.size());

    for (int l = 0; l < msg.data[k].data.size(); l++) {
      gp_eval_[k][l] = msg.data[k].data[l];
    }
  }

  gp_eval_received_ = true;
}


void ExperimentStatsNode::algae_cb(const mf_farm_simulator::Algae msg)
{
  algae_heatmaps_.resize(msg.algae.size());

  for (int k = 0; k < msg.algae.size(); k++) {
    const mf_farm_simulator::Alga *alga = &msg.algae[k];
    algae_heatmaps_[k].resize(alga->disease_heatmap.size());

    for (int l = 0; l < alga->disease_heatmap.size(); l++) {
      algae_heatmaps_[k][l].resize(alga->disease_heatmap[l].data.size());

      for (int m = 0; m < alga->disease_heatmap[l].data.size(); m++) {
        algae_heatmaps_[k][l][m] = alga->disease_heatmap[l].data[m];
      }
    }
  }

  algae_received_ = true;
}


}  // namespace mfcpp



/**
 * \brief  Main function called before node initialisation
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "experiment_stats");
  mfcpp::ExperimentStatsNode experiment_stats_node;
  experiment_stats_node.run_node();

  return 0;
}
