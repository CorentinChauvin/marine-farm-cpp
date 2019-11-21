/**
 * @file
 *
 * \brief  Declaration of a nodelet for path plannning of an underwater robot
 *         surveying a marine farm
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#ifndef PLANNING_NODELET_HPP
#define PLANNING_NODELET_HPP

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <csignal>

namespace mfcpp {

/**
 * \brief  Nodelet for path planning of an underwater robot surveying a marine
 *         farm
 */
class PlanningNodelet: public nodelet::Nodelet {
  public:
    PlanningNodelet();
    ~PlanningNodelet();

    /**
     * \brief  Function called at beginning of nodelet execution
     */
    virtual void onInit();

  private:
    // Static members
    // Note: the timers need to be static since stopped by the SIGINT callback
    static sig_atomic_t volatile b_sigint_;  ///<  Whether SIGINT signal has been received
    static ros::Timer main_timer_;   ///<  Timer callback for the main function

    // Private members
    ros::NodeHandle nh_;            ///<  Node handler (for topics and services)
    ros::NodeHandle private_nh_;    ///<  Private node handler (for parameters)

    /// \name  ROS parameters
    ///@{
    float main_freq_;         ///<  Frequency of the main loop
    ///@}

    /**
     * \brief  Main callback which is called by a timer
     *
     * \param timer_event  Timer event information
     */
    void main_cb(const ros::TimerEvent &timer_event);

    /**
     * \brief  SINGINT (Ctrl+C) callback to stop the nodelet properly
     */
    static void sigint_handler(int s);

};



}  // namespace mfcpp


#endif
