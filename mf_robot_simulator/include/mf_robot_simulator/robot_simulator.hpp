/**
 * @file
 *
 * \brief  Declaration of underwater robot simulator
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#ifndef ROBOT_SIMULATOR_HPP
#define ROBOT_SIMULATOR_HPP

#include "mf_robot_model/robot_model.hpp"
#include "mf_robot_simulator/Command.h"
#include "mf_robot_simulator/CartesianCommand.h"
#include <ros/ros.h>
#include <vector>


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
    ros::Subscriber cart_input_sub_;  ///< ROS subscriber for the debug cartesian control input
    ros::Publisher odom_pub_;    ///<  ROS publisher for the odometry output
    ros::Publisher rviz_pub_;    ///<  ROS publisher for rviz markers
    tf2_ros::TransformBroadcaster tf_br_;  ///<  Tf broadcaster

    RobotModel robot_model_;         ///<  Robot model
    RobotModel::state_type state_;   ///<  Current state of the robot
    RobotModel::input_type input_;   ///<  Current control input
    std::vector<float> cart_input_;  ///<  Current debug cartesian control input

    // ROS parameters
    float update_freq_;   ///<  State update frequency
    std::string fixed_frame_;  ///<  Frame in which the pose is expressed
    std::string robot_frame_;  ///<  Frame of the robot
    float robot_length_;  ///<  Length of the robot (assuming cylindrical shape)
    float robot_radius_;  ///<  Radius of the robot (assuming cylindrical shape)
    int nbr_int_steps_;   ///<  Initial number of integration steps during the update
    float bnd_delta_m_;   ///<  Bound on delta_m (ballast control)
    RobotModel::input_type bnd_input_;  ///<  Bounds on the control input

    /**
     * \brief  Callback for the control input
     */
    void input_cb(const mf_robot_simulator::Command::ConstPtr &msg);

    /**
     * \brief  Callback for the debug cartesian control input
     */
    void cart_input_cb(const mf_robot_simulator::CartesianCommand::ConstPtr &msg);

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
