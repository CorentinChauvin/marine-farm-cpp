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
    // Private members
    ros::NodeHandle nh_;         ///<  ROS node handler
    ros::Subscriber input_sub_;  ///< ROS subscriber for the control input
    ros::Publisher odom_publisher_;  ///<  ROS publisher for the odometry output
    tf2_ros::TransformBroadcaster tf_br_;  ///<  Tf broadcaster

    RobotModel robot_model_;        ///<  Robot model
    RobotModel::state_type state_;  ///<  Current state of the robot
    RobotModel::input_type input_;  ///<  Current control input

    // ROS parameters
    double update_freq_;  ///<  State update frequency
    int nbr_int_steps_;   ///<  Initial number of integration steps during the update
    double bnd_delta_m_;  ///<  Bound on delta_m (ballast control)
    RobotModel::input_type bnd_input_;  ///<  Bounds on the control input

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
