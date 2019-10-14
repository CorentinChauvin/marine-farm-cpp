/**
 * @file
 *
 * \brief  Declaration of underwater robot simulator
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#ifndef ROBOT_SIMULATOR_HPP
#define ROBOT_SIMULATOR_HPP

#include "robot_model/robot_model.hpp"
#include <ros/ros.h>


namespace mfcpp {

/**
 * \brief  Robot simulator
 */
class RobotSimulator {
  public:
    RobotSimulator();
    ~RobotSimulator();

    /**
     * \brief
     */
    void init_node();

    /**
     * \brief
     */
    void run_node();

  private:
    /// ROS node handler
    ros::NodeHandle nh_;

    // ros::Subscriber cones_absolute_subscriber_;
    // ros::Publisher pose_publisher_;

    /// Robot model
    RobotModel robot_model_;

};


}  // namespace mfcpp;

#endif
