/**
 * @file
 *
 * \brief  Definition of a node to generate a fake trajectory and evaluate the
 *         performance of the control.
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#include "clock_publisher.hpp"
#include <rosgraph_msgs/Clock.h>
#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <ctime>

using std::cout;
using std::endl;


namespace mfcpp {


ClockPublisherNode::ClockPublisherNode():
  nh_("~")
{
  init_node();
}


ClockPublisherNode::~ClockPublisherNode()
{

}


void ClockPublisherNode::init_node()
{
  // ROS parameters
  nh_.param<bool>("/use_sim_time", use_sim_time_, false);
  nh_.param<float>("publish_rate", publish_rate_, 1.0);
  nh_.param<float>("time_factor", time_factor_, 1.0);

  // Other variables


  // ROS publishers
  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 0);
}


void ClockPublisherNode::run_node()
{
  if (!use_sim_time_)
    return;

  ros::WallRate rate(publish_rate_ * time_factor_);
  double current_time = 0.0;

  while (ros::ok()) {
    rosgraph_msgs::Clock msg;
    msg.clock = ros::Time(current_time);
    clock_pub_.publish(msg);

    current_time += 1 / publish_rate_;
    rate.sleep();
  }
}


}  // namespace mfcpp



/**
 * \brief  Main function called before node initialisation
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "clock_publisher");
  mfcpp::ClockPublisherNode clock_publisher_node;
  clock_publisher_node.run_node();

  return 0;
}
