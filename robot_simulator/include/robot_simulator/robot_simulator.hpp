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
#include "robot_simulator/Command.h"
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
     * \brief  Initialise the node and its parameters
     */
    void init_node();

    /**
     * \brief  Run the simulator
     */
    void run_node();

    /**
     * \brief  Display the current state
     */
    void disp_state();

  private:
    /// ROS node handler
    ros::NodeHandle nh_;

    /// ROS subscriber for the control input
    ros::Subscriber input_sub_;

    /// ROS publisher for the odometry output
    ros::Publisher odom_publisher_;

    /// Tf broadcaster
    tf2_ros::TransformBroadcaster tf_br_;

    /// State update frequency
    double update_freq_;

    /// Initial number of integration steps during the update
    int nbr_int_steps_;

    /// Robot model
    RobotModel robot_model_;

    /// Current state of the robot
    RobotModel::state_type state_;

    /// Bound on delta_m (ballast control)
    double bnd_delta_m_;

    /// Current control input
    RobotModel::input_type input_;

    /// Bounds on the control input
    RobotModel::input_type bnd_input_;

    /**
     * \brief  Callback for the control input
     */
    void input_cb(const robot_simulator::Command::ConstPtr &msg);

    /**
     * \brief  Updates the robot state by integrating the ODE
     *
     * \param dt  Duration of the update
     */
    void update_state(float dt);

    /**
     * \brief  Publishes the robot state
     */
    void publish_state();

};


}  // namespace mfcpp;

#endif
