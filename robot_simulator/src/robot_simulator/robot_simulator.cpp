/**
 * @file
 *
 * \brief  Definition of underwater robot simulator node
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "robot_simulator.hpp"
#include "robot_model/robot_model.hpp"
#include "robot_simulator/Command.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ros/ros.h>

#include <vector>

using namespace std;


namespace mfcpp {

RobotSimulator::RobotSimulator():
  nh_("~")
{

}


RobotSimulator::~RobotSimulator()
{

}


void RobotSimulator::init_node()
{
  // ROS parameters
  vector<double> model_csts;  // model constants

  nh_.param<double>("update_freq", update_freq_, 1.0);
  nh_.param<int>("nbr_int_steps", nbr_int_steps_, 10);
  nh_.param<vector<double>>("model_constants", model_csts, vector<double>(11, 0.0));
  nh_.param<vector<double>>("init_state", state_, vector<double>(13, 0.0));
  nh_.param<double>("bnd_delta_m", bnd_delta_m_, 0.02);
  nh_.param<vector<double>>("bnd_input", bnd_input_, vector<double>(4, 0.0));

  // ROS publishers
  odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("odom_output", 1);

  // ROS subscribers
  input_sub_ = nh_.subscribe<robot_simulator::Command>("input", 1, &RobotSimulator::input_cb, this);

  // Simulator initialisation
  input_ = {0, 0, 0, 0};
  robot_model_ = RobotModel(model_csts);

}


void RobotSimulator::run_node()
{
  init_node();

  ros::Rate loop_rate(update_freq_);
  float dt = 1/update_freq_;

  while(ros::ok()) {
    ros::spinOnce();

    update_state(dt);
    publish_state();
    cout << "---" << endl;
    disp_state();

    loop_rate.sleep();
  }

  ros::spin();
}


void RobotSimulator::disp_state()
{
  cout << "Robot state: " << endl
       << "pos=(" << state_[0] << ", " << state_[1] << ", " << state_[2] << ")\n"
       << "rot=(" << state_[3] << ", " << state_[4] << ", " << state_[5] << ")\n"
       << "lin_speed=(" << state_[6] << ", " << state_[7] << ", " << state_[8] << ")\n"
       << "ang_speed=(" << state_[9] << ", " << state_[10] << ", " << state_[11] << ")\n"
       << "delta_m=" << state_[12] << endl;
}


void RobotSimulator::input_cb(const robot_simulator::Command::ConstPtr &msg)
{
  input_[0] = msg->n;
  input_[1] = msg->delta_r;
  input_[2] = msg->delta_e;
  input_[3] = msg->P;

  for (int i = 0; i < input_.size(); i++) {
    if (abs(input_[i]) > bnd_input_[i])
      input_[i] = copysign(bnd_input_[i], input_[i]);
  }
}


void RobotSimulator::update_state(float dt)
{
  // Integrate the model
  robot_model_.integrate(state_, input_, 0.0, dt, dt/nbr_int_steps_);

  // Bound the ballast state
  if (abs(state_[12]) >= bnd_delta_m_) {
    state_[12] = copysign(bnd_delta_m_, state_[12]);
  }
}


void RobotSimulator::publish_state()
{
  // Publish odometry
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "ocean";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = state_[0];
  odom.pose.pose.position.y = state_[1];
  odom.pose.pose.position.z = state_[2];

  tf::Quaternion quat;
	quat.setRPY(state_[3], state_[4], state_[5]);
	tf::quaternionTFToMsg(quat, odom.pose.pose.orientation);

  odom.twist.twist.linear.x = state_[6];
  odom.twist.twist.linear.y = state_[7];
  odom.twist.twist.linear.z = state_[8];
  odom.twist.twist.angular.x = state_[9];
  odom.twist.twist.angular.y = state_[10];
  odom.twist.twist.angular.z = state_[11];

  odom_publisher_.publish(odom);

  // Broadcast transform
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = ros::Time::now();
  transform.header.frame_id = "ocean";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = odom.pose.pose.position.x;
  transform.transform.translation.y = odom.pose.pose.position.y;
  transform.transform.translation.z = odom.pose.pose.position.z;
  transform.transform.rotation.x = odom.pose.pose.orientation.x;
  transform.transform.rotation.y = odom.pose.pose.orientation.y;
  transform.transform.rotation.z = odom.pose.pose.orientation.z;
  transform.transform.rotation.w = odom.pose.pose.orientation.w;
  tf_br_.sendTransform(transform);
}



}  // namespace mfcpp


/**
 * \brief  Main function called before node initialisation
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_simulator");
  mfcpp::RobotSimulator robot_simulator;
  robot_simulator.run_node();
  return 0;
}
