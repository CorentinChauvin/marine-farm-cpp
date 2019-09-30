/**
 * @file
 *
 * \brief  Declaration of nodelet for managing the farm simulation
 * \author Corentin Chauvin-Hamea
 * \date   2019
 */

#ifndef FARM_NODELET_HPP
#define FARM_NODELET_HPP

#include "farm_common.hpp"
#include "farm_simulator/FarmSimulatorConfig.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <string>
#include <vector>


namespace mfcpp {
  /**
   * \brief  Nodelet class for farm simulation
   */
  class FarmNodelet: public nodelet::Nodelet {
    public:
      FarmNodelet();
      ~FarmNodelet();

      /**
       * \brief  Function called at beginning of nodelet execution
       */
      virtual void onInit();

    private:
      // Private members
      ros::NodeHandle nh_;            ///<  Node handler (for topics and services)            )
      ros::NodeHandle private_nh_;    ///<  Private node handler (for parameters
      ros::Publisher rviz_pub_;       ///<  ROS publisher for Rviz
      std::vector<AlgaeLine> algae_lines_;  ///<  Vector of all the algae in the farm
      bool reconfigure_initialised_;  ///<  Whether the dynamic reconfigure callback
                                      ///<  has been called once

      // ROS parameters
      float main_loop_freq_;  ///<  Frequency of the main loop
      int nbr_lines_;       ///<  Number of algae lines
      float offset_lines_;  ///<  Lateral distance (m) between each line
      float length_lines_;  ///<  Length (m) of each line
      float thickness_lines_;  ///<  Diameter (m) of each line
      float depth_lines_;   ///<  Distance (m) between water surface and line
      float depth_water_;   ///<  Distance (m) between water surface and seafloor
      float anchors_diameter_;  ///<  diameter of the cylindrical anchors
      float anchors_height_;    ///<  height of the cylindrical anchors

      /**
       * \brief  Main loop of the nodelet
       */
      void run_nodelet();

      /**
       * \brief  Callback for dynamic reconfigure
       *
       * \param  New configuration
       * \param  Change level
       */
      void reconfigure_callback(farm_simulator::FarmSimulatorConfig &config,
        uint32_t level);

      /**
       * \brief  Initialise the algae lines
       *
       * \param randomise  Whether to randomise the position of each line
       * \param phi        (optional) First spherical angle for the first point
       *                   of each line
       * \param theta      (optional) Second spherical angle for the first point
       *                   of each line
       */
      void init_algae_lines(bool randomise, float phi=0.0, float theta=0.0);

      /**
       * \brief  Displays objects by publishing Rviz markers
       *
       * \param duration  Duration of the marker (in sec)
       */
      void pub_rviz_markers(float duration) const;

  };



}  // namespace mfcpp

#endif
