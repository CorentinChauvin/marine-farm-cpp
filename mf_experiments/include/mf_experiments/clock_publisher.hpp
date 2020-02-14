/**
 * @file
 *
 * \brief  To publish on the /clock topic at some frequency
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#ifndef CLOCK_PUBLISHER_HPP
#define CLOCK_PUBLISHER_HPP

#include <ros/ros.h>



namespace mfcpp {

/**
 * \brief  Class to publish on the /clock topic
 *
 * Publishes simulation time at some user specified rate. Won't publish if
 * /use_sim_time ROS parameter is not set to true.
 */
class ClockPublisherNode {
  public:
    ClockPublisherNode();
    ~ClockPublisherNode();

    /**
     * \brief  Runs the node
     */
    void run_node();

  private:
    // Private members
    ros::NodeHandle nh_;        ///<  Node handler
    ros::Publisher clock_pub_;  ///<  Clock publisher

    /// \name  ROS parameters
    ///@{
    bool use_sim_time_;  ///<  Whether to publish something in the /clock topic
    float publish_rate_; ///<  Frequency (Hz) of publish on the /clock topic (ROS frequency)
    float time_factor_;  ///<  Time factor of the simulation (1.0 for same as wall time)
    ///@}


    /**
     * \brief  Initialises the node and its parameters
     */
    void init_node();
};


}  // namespace mfcpp

#endif
