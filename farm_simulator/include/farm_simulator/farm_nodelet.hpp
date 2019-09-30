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
      /**
       * \brief  Arguments used to create a Rviz maker
       */
      struct MarkerArgs
      {
        ros::Time stamp;       ///<  Time stamp for the ROS message
        std::string frame_id;  ///<  Frame in which position/orientation of the object is
        std::string ns;        ///<  Namespace for the Rviz marker
        ros::Duration duration;  ///<  Duration of the marker (in sec)
      };

      // Private members
      ros::NodeHandle nh_;            ///<  Node handler (for topics and services)            )
      ros::NodeHandle private_nh_;    ///<  Private node handler (for parameters
      ros::Publisher rviz_pub_;       ///<  ROS publisher for Rviz
      std::vector<AlgaeLine> algae_lines_;  ///<  Vector of all the algae in the farm

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
       * \brief  Initialise the algae lines
       */
      void init_algae_lines();

      /**
       * \brief  Displays objects by publishing Rviz markers
       *
       * \param duration  Duration of the marker (in sec)
       */
      void pub_rviz_markers(float duration) const;


      // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
      // TODO: move in an other file (display_rviz.hpp?)

      /**
       * \brief  Creates a Rviz marker to display a line
       *
       * \param p1            First extremity of the line
       * \param p2            Second extremity of the line
       * \param thickness     Thickness of the line (in m)
       * \param common_args   Common arguments to fill ROS message
       * \return out_marker   Corresponding Rviz marker
       */
      visualization_msgs::Marker rviz_marker_line(tf2::Vector3 p1, tf2::Vector3 p2,
        float thickness, const MarkerArgs &common_args) const;

      /**
       * \brief  Creates a Rviz marker to display a cylinder
       *
       * The cylinder will be aligned with the z axis of the frame provided
       * in `common_args`.
       *
       * \param p             Position of the center of the cylinder
       * \param diameter      Diameter (in m)
       * \param height        Height (in m)
       * \param common_args   Common arguments to fill ROS message
       * \return out_marker   Corresponding Rviz marker
       */
      visualization_msgs::Marker rviz_marker_cylinder(tf2::Vector3 p, float diameter,
        float height, const MarkerArgs &common_args) const;

      /**
       * \brief  Populates id of all the markers in a marker array
       *
       * \note  The id of each marker needs to be different for Rviz to display it
       * correctly.
       */
      void pop_marker_ids(visualization_msgs::MarkerArray &array) const;

  };



} // namespace mfcpp

#endif
