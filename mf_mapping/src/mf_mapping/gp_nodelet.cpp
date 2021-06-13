/**
 * @file
 *
 * \brief  Definition of a nodelet for a Gaussian Process mapping an
 *         algae wall
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "gp_nodelet.hpp"
#include "mf_common/Array2D.h"
#include "mf_common/Float32Array.h"
#include "mf_mapping/UpdateGP.h"
#include "mf_mapping/LoadGP.h"
#include "mf_sensors_simulator/CameraOutput.h"
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>
#include <iostream>

using namespace std;

PLUGINLIB_EXPORT_CLASS(mfcpp::GPNodelet, nodelet::Nodelet)


namespace mfcpp {

/*
 * Definition of static variables
 */
sig_atomic_t volatile GPNodelet::b_sigint_ = 0;
ros::Timer GPNodelet::main_timer_ = ros::Timer();

/*
 * Definition of member functions
 */
GPNodelet::GPNodelet():
  tf_listener_(tf_buffer_)
{

}

GPNodelet::~GPNodelet() {}


void GPNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Catch SIGINT (Ctrl+C) stop signal
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = GPNodelet::sigint_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // ROS parameters
  private_nh_.param<float>("main_freq", main_freq_, 1.0);
  private_nh_.param<string>("wall_frame", wall_frame_, "wall");
  private_nh_.param<string>("camera_frame", camera_frame_, "camera");
  private_nh_.param<float>("camera_var", camera_var_, 1.0);
  private_nh_.param<float>("camera_decay", camera_decay_, 1.0);
  private_nh_.param<float>("matern_length", matern_length_, 1.0);
  private_nh_.param<float>("matern_var", matern_var_, 1.0);
  private_nh_.param<float>("matern_thresh", matern_thresh_, 0.001);
  private_nh_.param<float>("gp_init_mean", gp_init_mean_, 1.0);
  private_nh_.param<float>("gp_noise_var", gp_noise_var_, 1.0);
  private_nh_.param<float>("gp_cov_thresh", gp_cov_thresh_, 0.0);
  private_nh_.param<float>("out_scale", out_scale_, 1.0);
  private_nh_.param<float>("size_wall_x", size_wall_x_, 2.0);
  private_nh_.param<float>("size_wall_y", size_wall_y_, 30.0);
  private_nh_.param<int>("size_gp_x", size_gp_x_, 2);
  private_nh_.param<int>("size_gp_y", size_gp_y_, 30);
  private_nh_.param<int>("size_img_x", size_img_x_, 40);
  private_nh_.param<int>("size_img_y", size_img_y_, 600);

  // Other variables
  camera_msg_available_ = false;
  tf_got_ = false;
  gp_initialised_ = false;
  gp_loaded_ = false;
  delta_x_ = size_wall_x_ / (size_gp_x_-1);
  delta_y_ = size_wall_y_ / (size_gp_y_-1);
  size_gp_ = size_gp_x_ * size_gp_y_;
  size_img_ = size_img_x_ * size_img_y_;
  out_values_.resize(size_img_, 0.5);
  changed_pxl_.resize(size_img_, false);

  radius_obs_ = -matern_length_/sqrt(3)
              * (lambert_wm1(-matern_thresh_/(exp(1)*matern_var_)) + 1);

  // ROS subscribers
  camera_sub_ = nh_.subscribe<mf_sensors_simulator::CameraOutput>("camera_out", 1, &GPNodelet::camera_cb, this);

  // ROS publishers
  wall_img_pub_ = nh_.advertise<sensor_msgs::Image>("gp_wall_img", 0);
  cov_img_pub_ = nh_.advertise<sensor_msgs::Image>("gp_cov_img", 0);
  gp_mean_pub_ = nh_.advertise<mf_common::Float32Array>("gp_mean", 0);
  gp_cov_pub_ = nh_.advertise<mf_common::Array2D>("gp_cov", 0);
  gp_eval_pub_ = nh_.advertise<mf_common::Array2D>("gp_eval", 0);

  // ROS services
  update_gp_serv_ = nh_.advertiseService("update_gp", &GPNodelet::update_gp_cb, this);
  load_gp_serv_ = nh_.advertiseService("load_gp", &GPNodelet::load_gp_cb, this);


  // Main loop
  main_timer_ = private_nh_.createTimer(
    ros::Duration(1/main_freq_), &GPNodelet::main_cb, this
  );
}


void GPNodelet::main_cb(const ros::TimerEvent &timer_event)
{
  if (!ros::ok() || ros::isShuttingDown() || b_sigint_)
    return;

  tf_got_ = get_tf();

  if (!gp_initialised_) {
    NODELET_INFO("[gp_nodelet] Initialisation of Gaussian Process...");
    init_gp();
    gp_initialised_ = true;

    NODELET_INFO("[gp_nodelet] Gaussian Process initialised.");
  }
  else if (camera_msg_available_) {
    if (gp_loaded_) {
      // Don't update using measurement, the GP has been manually loaded
      gp_loaded_ = false;
    } else {
      // Prepare input data
      vector<float> x, y, z;
      transform_points(camera_msg_->x, camera_msg_->y, camera_msg_->z,
        x, y, z, camera_msg_->header.frame_id, wall_frame_);

      vector<float> distances(x.size());
      for (unsigned int k = 0; k < x.size(); k++) {
        distances[k] = Eigen::Vector3f(camera_msg_->x[k],
                                       camera_msg_->y[k],
                                       camera_msg_->z[k]).norm();
      }

      // Update the Gaussian Process
      RectArea obs_coord;  // coordinates of the rectangular area of the observed state
      update_gp(x, y, z, distances, camera_msg_->value, gp_mean_, gp_cov_, idx_obs_,
        obs_coord);

      notif_changing_pxls(obs_coord);  // notified changed pixels
    }

    // Publish output
    publish_gp_state();  // publish mean and covariance of the GP
    publish_gp_eval();   // publish evaluated GP and covariance

    camera_msg_available_ = false;
  }
}


void GPNodelet::sigint_handler(int s) {
  b_sigint_ = 1;
  main_timer_.stop();

  raise(SIGTERM);
}


void GPNodelet::camera_cb(const mf_sensors_simulator::CameraOutput::ConstPtr &msg)
{
  if (msg->x.size() > 0) {
    camera_msg_ = msg;
    camera_msg_available_ = true;
  }
}


bool GPNodelet::get_tf()
{
  geometry_msgs::TransformStamped transform;

  try {
    transform = tf_buffer_.lookupTransform(wall_frame_, camera_frame_, ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    NODELET_WARN("[gp_nodelet] %s", ex.what());
    return false;
  }

  wall_camera_tf_ = transform;
  return true;
}


bool GPNodelet::update_gp_cb(mf_mapping::UpdateGP::Request &req,
  mf_mapping::UpdateGP::Response &res)
{
  // Check input data validity and initialisation
  if (req.update_mean && !req.use_internal_mean && req.mean.size() != size_gp_) {
    NODELET_WARN("[gp_nodelet] Input mean of the wrong size");
    return false;
  }

  if (!req.use_internal_cov && req.cov.size() != size_gp_) {
    NODELET_WARN("[gp_nodelet] Input covariance of the wrong size");
    return false;
  }

  if (!tf_got_)
    return false;

  // Parse the input GP mean and covariance
  Eigen::VectorXf mean(size_gp_);
  Eigen::MatrixXf cov(size_gp_, size_gp_);

  if (!req.update_mean || req.use_internal_mean)
    mean = gp_mean_;
  else {
    for (int k = 0; k < size_gp_; k++) {
      mean(k) = req.mean[k];
    }
  }

  if (req.use_internal_cov)
    cov = gp_cov_;
  else {
    for (int i = 0; i < size_gp_; i++) {
      for (int j = 0; j <= i; j++) {
        cov(i, j) = req.cov[i].data[j];
        cov(j, i) = cov(i, j);
      }
    }
  }

  // Get wall to camera ROS transform
  geometry_msgs::TransformStamped wall_camera_tf;

  try {
    wall_camera_tf = tf_buffer_.lookupTransform(wall_frame_,  camera_frame_, req.stamp);
  }
  catch (tf2::TransformException &ex) {
    NODELET_WARN("[gp_nodelet] %s", ex.what());
    return false;
  }

  // Parse the input sets of measurements
  int n_meas = req.meas.size();  // number of sets of measurements
  vector<vector<float>> x_meas(n_meas), y_meas(n_meas), z_meas(n_meas);
  vector<vector<float>> values(n_meas), distances(n_meas);

  for (int k = 0; k < n_meas; k++) {
    vector<float> x = req.meas[k].x;
    vector<float> y = req.meas[k].y;
    vector<float> z = req.meas[k].z;
    transform_points(x, y, z, x_meas[k], y_meas[k], z_meas[k], wall_camera_tf);

    values[k] = req.meas[k].value;
    distances[k] = req.meas[k].distance;
  }

  // Update and evaluate the given Gaussian Processes
  res.new_mean.resize(n_meas);
  res.new_cov.resize(n_meas);
  res.new_cov_diag.resize(n_meas);
  res.eval_values.resize(n_meas);

  vector<unsigned int> idx_obs;  // array of corresponding indices for obs states
  RectArea obs_coord;  // (won't be used)

  for (int k = 0; k < n_meas; k++) {
    // Update and evaluate the GP
    update_gp(x_meas[k], y_meas[k], z_meas[k], distances[k], values[k],
      mean, cov, idx_obs, obs_coord, req.update_mean);

    if (req.eval_gp) {
      int size_meas = x_meas[k].size();
      res.eval_values[k].data.resize(size_meas);
      Eigen::VectorXf x_obs, y_obs, W;  // necessary objects for evaluation
      prepare_eval(idx_obs, mean, x_obs, y_obs, W);

      for (int l = 0; l < size_meas; l++) {
        res.eval_values[k].data[l] = eval_gp(x_meas[k][l], y_meas[k][l],
          x_obs, y_obs, W);
      }
    }

    // Fill the outputs
    if (req.update_mean) {
      res.new_mean[k].data.resize(size_gp_);

      for (int l = 0; l < size_gp_; l++) {
        res.new_mean[k].data[l] = mean(l);
      }
    }

    if (req.return_cov_diag) {
      res.new_cov_diag[k].data.resize(size_gp_);

      for (int l = 0; l < size_gp_; l++) {
        res.new_cov_diag[k].data[l] = cov(l, l);
      }
    } else {
      res.new_cov[k].data.resize(size_gp_);

      for (int i = 0; i < size_gp_; i++) {
        res.new_cov[k].data[i].data.resize(size_gp_);

        for (int j = 0; j <= size_gp_; j++) {
          res.new_cov[k].data[i].data[j] = cov(i, j);
        }
      }
    }
  }

  return true;
}


bool GPNodelet::load_gp_cb(mf_mapping::LoadGP::Request &req,
  mf_mapping::LoadGP::Response &res)
{
  // Check GP has been initialised
  if (!gp_initialised_) {
    res.gp_not_yet_init = true;
    res.gp_loaded = false;
    return true;
  } else {
    res.gp_not_yet_init = false;
  }

  // Check input validity
  if (req.mean.size() != size_gp_) {
    ROS_WARN("[gp_nodelet] Trying to load the GP with the wrong size (given: %d, expected: %d)",
      (int)req.mean.size(), size_gp_);

    res.gp_loaded = false;
    return true;
  }

  // Set GP mean and covariance
  for (int k = 0; k < size_gp_; k++) {
    gp_mean_[k] = req.mean[k];
  }

  if (req.init_cov) {
    gp_cov_ = Eigen::MatrixXf::Zero(size_gp_, size_gp_);

    for (unsigned int k = 0; k < size_gp_; k++) {
      gp_cov_(k, k) = 1/gp_noise_var_;
    }
  }

  // Notify changed pixels (all pixels)
  for (int k = 0; k < changed_pxl_.size(); k++) {
    changed_pxl_[k] = true;
  }

  idx_obs_.resize(size_gp_);
  for (int k = 0; k < size_gp_; k++) {
    idx_obs_[k] = k;
  }

  // Return values
  gp_loaded_ = true;
  res.gp_loaded = true;
  return true;
}


bool GPNodelet::transform_points(const vec_f &x_in, const vec_f &y_in, const vec_f &z_in,
  vec_f &x_out, vec_f &y_out, vec_f &z_out, string frame_in, string frame_out)
{
  // Get the tf transform
  geometry_msgs::TransformStamped transform;

  try {
    transform = tf_buffer_.lookupTransform(frame_out, frame_in, ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    NODELET_WARN("[gp_nodelet] %s",ex.what());
    return false;
  }

  // Transform the points
  return transform_points(x_in, y_in, z_in, x_out, y_out, z_out, transform);
}


bool GPNodelet::transform_points(const vec_f &x_in, const vec_f &y_in, const vec_f &z_in,
  vec_f &x_out, vec_f &y_out, vec_f &z_out, const geometry_msgs::TransformStamped &transform)
{
  unsigned int n = x_in.size();
  x_out.resize(n);
  y_out.resize(n);
  z_out.resize(n);

  for (unsigned int k = 0; k < n; k++) {
    geometry_msgs::Pose p_in, p_out;
    p_in.position.x = x_in[k];
    p_in.position.y = y_in[k];
    p_in.position.z = z_in[k];

    tf2::doTransform(p_in, p_out, transform);

    x_out[k] = p_out.position.x;
    y_out[k] = p_out.position.y;
    z_out[k] = p_out.position.z;
  }

  return true;
}


double GPNodelet::lambert_w0(double z, double precision, double init_w) const
{
  // Handle out of bound case
  if (z < 0) {
    NODELET_WARN("[gp_nodelet] Calling lambert_w0 with z=%f", z);
    return 0;
  }

  // Apply Newton's method
  return lambert_w(z, precision, init_w);
}


double GPNodelet::lambert_wm1(double z, double precision, double init_w) const
{
  // Handle out of bound case
  if (z < -1/exp(1) || z >= 0) {
    NODELET_WARN("[gp_nodelet] Calling lambert_wm1 with z=%f", z);
    return 0;
  }

  // Apply Newton's method
  return lambert_w(z, precision, init_w);
}


double GPNodelet::lambert_w(double z, double precision, double init_w) const
{
  double w = init_w;
  double last_w = w;
  double delta;
  int safe_count = 100;

  do {
    w = w - (w*exp(w) - z) / (exp(w) + w*exp(w));
    delta = abs(last_w - w);
    last_w = w;
  } while (delta > precision && safe_count-- > 0);

  if (safe_count == 0)
    NODELET_WARN("[gp_nodelet] lambert_w0(%f) fails to converge, throwing intermediary result w=%f anyway.", z, w);

  return w;
}





}  // namespace mfcpp
