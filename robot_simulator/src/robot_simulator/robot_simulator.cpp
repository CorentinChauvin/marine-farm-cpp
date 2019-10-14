/**
 * @file
 *
 * \brief  Definition of underwater robot simulator node
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "robot_simulator.hpp"
#include "robot_model/robot_model.hpp"
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
  vector<double> c;  // model constants

  nh_.param<vector<double>>("model_constants", c, vector<double>(11, 0.0));

  // ROS publishers
  // pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_output", 1);

  // ROS subscribers
  // odometry_subscriber_ = nh_.subscribe<nav_msgs::Odometry>("odom_input", 1, &Fastslam2Node::odometry_callback, this);

    // Robot model initialisation
    robot_model_ = RobotModel(c);

}


void RobotSimulator::run_node()
{
  init_node();

  cout << "Simulator node started!!" << endl;

  RobotModel::state_type state(12);
  RobotModel::input_type input = {1, 2, 3, 4};

  robot_model_.integrate(state, input, 0.0, 1.0, 0.1);

  init_node();
  ros::spin();
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
