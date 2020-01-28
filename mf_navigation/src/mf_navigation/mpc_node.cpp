/**
 * @file
 *
 * \brief  Definition of a node for Model Predictive Control of an underwater
 *         robot
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#include "mpc_node.hpp"
#include "mf_common/common.hpp"
#include "mf_robot_model/robot_model.hpp"
#include "mf_common/Float32Array.h"
#include "mf_robot_simulator/Command.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>

using namespace std;
using Eigen::MatrixXd;
using Eigen::DiagonalMatrix;


namespace mfcpp {


MPCNode::MPCNode():
  nh_("~"),
  tf_listener_(tf_buffer_)
{

}


MPCNode::~MPCNode() {}


void MPCNode::init_node()
{
  // ROS parameters
  vector<double> model_csts;  // model constants
  vector<double> P, Q_x, R_u, R_delta;  // MPC tuning parameters

  nh_.param<float>("main_freq", main_freq_, 10.0);
  nh_.param<string>("fixed_frame", fixed_frame_, "ocean");
  nh_.param<float>("desired_speed", desired_speed_, 1.0);
  nh_.param<float>("time_horizon", time_horizon_, 1.0);
  nh_.param<int>("nbr_steps", nbr_steps_, 10);
  nh_.param<bool>("disable_vbs", disable_vbs_, false);
  nh_.param<vector<double>>("P", P, vector<double>(13, 1.0));
  nh_.param<vector<double>>("Q_x", Q_x, vector<double>(13, 1.0));
  nh_.param<vector<double>>("R_u", R_u, vector<double>(4, 1.0));
  nh_.param<vector<double>>("R_delta", R_delta, vector<double>(4, 1.0));

  nh_.param<vector<double>>("model_constants", model_csts, vector<double>(11, 0.0));
  nh_.param<double>("bnd_delta_m", bounds_.delta_m, 1.0);
  nh_.param<vector<double>>("bnd_input", bounds_.input, vector<double>(4, 0.0));


  // Other variables
  robot_model_ = RobotModel(model_csts);
  path_received_ = false;
  state_received_ = false;

  last_desired_speed_ = desired_speed_;
  last_control_ = vector<float>(4, 0);

  if (disable_vbs_)
    bounds_.delta_m = 0;


  fill_diag_mat(P, tuning_params_.P);
  fill_diag_mat(Q_x, tuning_params_.Q_x);
  fill_diag_mat(R_u, tuning_params_.R_u);
  fill_diag_mat(R_delta, tuning_params_.R_delta);

  // ROS subscribers
  path_sub_ = nh_.subscribe<nav_msgs::Path>("path", 1, &MPCNode::path_cb, this);
  state_sub_ = nh_.subscribe<mf_common::Float32Array>("state", 1, &MPCNode::state_cb, this);

  // ROS publishers
  command_pub_ = nh_.advertise<mf_robot_simulator::Command>("command", 0);
  expected_traj_pub_ = nh_.advertise<geometry_msgs::PoseArray>("expected_traj", 0);
}


void MPCNode::run_node()
{
  init_node();

  ros::Rate loop_rate(main_freq_);

  while (ros::ok()) {
    ros::spinOnce();

    if (path_received_ && state_received_) {
      vector<float> control;
      bool control_computed;
      float desired_speed = desired_speed_;
      geometry_msgs::PoseArray expected_traj;

      control_computed = compute_control(
        desired_speed,
        last_desired_speed_,
        control,
        expected_traj
      );

      last_desired_speed_ = desired_speed;

      if (control_computed) {
        if (disable_vbs_)
          control[3] = 0;

        mf_robot_simulator::Command msg;
        msg.n = control[0];
        msg.delta_r = control[1];
        msg.delta_e = control[2];
        msg.P = control[3];

        command_pub_.publish(msg);

        last_control_ = control;


        expected_traj.header.frame_id = fixed_frame_;
        expected_traj_pub_.publish(expected_traj);
      }
    }

    loop_rate.sleep();
  }
}


void MPCNode::path_cb(const nav_msgs::Path msg)
{
  path_ = msg;
  path_received_ = true;
}


void MPCNode::state_cb(const mf_common::Float32Array msg)
{
  state_ = msg.array;
  state_received_ = true;
}


}  // namespace mfcpp


/**
 * \brief  Main function called before node initialisation
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc");
  mfcpp::MPCNode mpc_node;
  mpc_node.run_node();

  return 0;
}
