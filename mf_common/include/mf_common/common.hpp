/**
 * @file
 *
 * \brief  Declaration of common functions
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#ifndef COMMON_HPP
#define COMMON_HPP

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace mfcpp {

/**
 * \brief  Converts a TransformStamped message to a Pose message
 */
inline geometry_msgs::Pose to_pose(const geometry_msgs::TransformStamped &transform)
{
  geometry_msgs::Pose pose;
  pose.position.x = transform.transform.translation.x;
  pose.position.y = transform.transform.translation.y;
  pose.position.z = transform.transform.translation.z;
  pose.orientation.x = transform.transform.rotation.x;
  pose.orientation.y = transform.transform.rotation.y;
  pose.orientation.z = transform.transform.rotation.z;
  pose.orientation.w = transform.transform.rotation.w;

  return pose;
}

/**
 * \brief  Computes the square of the euclidean distance between two positions
 */
inline float distance2(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
  return pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2) + pow(p1.z-p2.z, 2);
}

/**
 * \brief  Computes the the euclidean distance between two positions
 */
inline float distance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
  return sqrt(distance2(p1, p2));
}

/**
 * \brief  Computes the the euclidean distance between the position of two poses
 */
inline float distance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2)
{
  return distance(p1.position, p2.position);
}

/**
 * \brief  Interpolates linearily two poses
 *
 * \param pose1  First pose
 * \param pose2  Second pose
 * \param t      Interpolation coefficient
 * \return  (1-t)*p1 + t*p2
 */
inline geometry_msgs::Pose interpolate(
  const geometry_msgs::Pose &pose1,
  const geometry_msgs::Pose &pose2,
  float t)
{
  // Prepare interpolation
  tf2::Vector3 p1, p2;
  tf2::convert(pose1.position, p1);
  tf2::convert(pose2.position, p2);

  tf2::Quaternion q1, q2;
  tf2::convert(pose1.orientation, q1);
  tf2::convert(pose2.orientation, q2);

  geometry_msgs::Pose new_pose;
  tf2::toMsg(   p1 + (p2-p1) * t,              new_pose.position);
  tf2::convert((q1 + (q2-q1) * t).normalize(), new_pose.orientation);

  return new_pose;
}


}  // namespace mfcpp

#endif
