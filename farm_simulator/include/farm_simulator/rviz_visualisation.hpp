/**
 * @file
 *
 * \brief  Declaration of common Rviz display functions
 * \author Corentin Chauvin-Hamea
 * \date   2019
 */

#ifndef RVIZ_VISUALISATION_HPP
#define RVIZ_VISUALISATION_HPP

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Vector3.h>
#include <ros/ros.h>


namespace mfcpp {
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

  /**
   * \brief  Fill header of marker
   *
   * \param marker        The marker to fill
   * \param common_args   Common arguments to fill ROS message
   */
  void fill_marker_header(visualization_msgs::Marker &marker,
    const MarkerArgs &common_args);

  /**
   * \brief  Creates a blank Rviz marker to display lines
   *
   * \param thickness     Thickness of the line (in m)
   * \param common_args   Common arguments to fill ROS message
   * \return  Corresponding Rviz marker
   */
  visualization_msgs::Marker rviz_marker_line(float thickness,
    const MarkerArgs &common_args);

  /**
   * \brief  Creates a Rviz marker to display a line
   *
   * \param p1            First extremity of the line
   * \param p2            Second extremity of the line
   * \param thickness     Thickness of the line (in m)
   * \param common_args   Common arguments to fill ROS message
   * \return  Corresponding Rviz marker
   */
  visualization_msgs::Marker rviz_marker_line(tf2::Vector3 p1, tf2::Vector3 p2,
    float thickness, const MarkerArgs &common_args);

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
   * \return  Corresponding Rviz marker
   */
  visualization_msgs::Marker rviz_marker_cylinder(tf2::Vector3 p, float diameter,
    float height, const MarkerArgs &common_args);

  /**
   * \brief  Creates a blank Rviz marker to display rectangles
   *
   * The rectangles will be made of two triangles
   *
   * \param common_args   Common arguments to fill ROS message
   * \return  Corresponding Rviz marker
   */
  visualization_msgs::Marker rviz_marker_rect(const MarkerArgs &common_args);

  /**
   * \brief  Creates a Rviz marker to display a rectangle
   *
   * The rectangle will be made of two triangles
   *
   * \param p1            First point
   * \param p2            Second point
   * \param p3            Third point
   * \param p4            Fourth point
   * \param common_args   Common arguments to fill ROS message
   * \return  Corresponding Rviz marker
   */
  visualization_msgs::Marker rviz_marker_rect(tf2::Vector3 p1, tf2::Vector3 p2,
    tf2::Vector3 p3, tf2::Vector3 p4, const MarkerArgs &common_args);

  /**
   * \brief  Populates id of all the markers in a marker array
   *
   * \note  The id of each marker needs to be different for Rviz to display it
   * correctly.
   */
  void pop_marker_ids(visualization_msgs::MarkerArray &array);

}  // namespace mfcpp

#endif
