/**
 * @file
 *
 * \brief  Declaration of common functions
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#ifndef COMMON_HPP
#define COMMON_HPP

#include "mf_common/EulerPose.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Dense>


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
 * \brief  Brings back an angle into [-pi, pi)
 */
template <class T>
inline T modulo_2pi(T angle)
{
  while (angle >= M_PI && angle < -M_PI) {
    if (angle >= M_PI)
      angle -= 2*M_PI;
    else
      angle += 2*M_PI;
  }

  return angle;
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

/**
 * \brief  Converts a quaternion to Roll-Pitch-Yaw Euler angles
 *
 * \param[in]  quat   Quaternion to convert
 * \param[out] roll   Resulting roll angle
 * \param[out] pitch  Resulting pitch angle
 * \param[out] yaw    Resulting yaw angle
 */
inline void to_euler(const tf2::Quaternion &quat, double &roll, double &pitch, double &yaw)
{
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

/**
 * \brief  Converts a quaternion to Roll-Pitch-Yaw Euler angles
 *
 * \param[in]  quat   Quaternion to convert
 * \param[out] roll   Resulting roll angle
 * \param[out] pitch  Resulting pitch angle
 * \param[out] yaw    Resulting yaw angle
 */
inline void to_euler(const geometry_msgs::Quaternion &quat, double &roll, double &pitch, double &yaw)
{
  tf2::Quaternion _quat(quat.x, quat.y, quat.z, quat.w);
  to_euler(_quat, roll, pitch, yaw);
}

/**
 * \brief  Converts Roll-Pitch-Yaw Euler angles to a quaternion
 *
 * \param[in]  roll   Roll angle to convert
 * \param[in]  pitch  Pitch angle to convert
 * \param[in]  yaw    Yaw angle to convert
 * \param[out] quat   Resulting quaternion
 */
inline void to_quaternion(double roll, double pitch, double yaw, tf2::Quaternion &quat)
{
  quat.setRPY(roll, pitch, yaw);
}

/**
 * \brief  Converts Roll-Pitch-Yaw Euler angles to a quaternion
 *
 * \param[in]  roll   Roll angle to convert
 * \param[in]  pitch  Pitch angle to convert
 * \param[in]  yaw    Yaw angle to convert
 * \param[out] quat   Resulting quaternion
 */
inline void to_quaternion(double roll, double pitch, double yaw, geometry_msgs::Quaternion &quat)
{
  tf2::Quaternion _quat;
  _quat.setRPY(roll, pitch, yaw);
  tf2::convert(_quat, quat);
}

/**
 * \brief  Converts Axis-angle to a quaternion
 *
 * \param[in]  x      X coordinate of the axis vector
 * \param[in]  y      Y coordinate of the axis vector
 * \param[in]  z      Z coordinate of the axis vector
 * \param[in]  angle  Angle around the axis vector
 * \param[out] quat   Resulting quaternion
 */
inline void to_quaternion(
  float x, float y, float z, float angle,
  geometry_msgs::Quaternion &quat)
{
  tf2::Vector3 vect(x, y, z);
  tf2::Quaternion _quat(vect, angle);
  tf2::convert(_quat, quat);
}

/**
 * \brief  Get orientation vector (x vector of the frame) given by Roll-Pitch-Yaw
 *
 * VectorT can be either Eigen::Vector3f or Eigen::Vector3d
 *
 * \param[in]  roll          Roll angle to convert
 * \param[in]  pitch         Pitch angle to convert
 * \param[in]  yaw           Yaw angle to convert
 * \param[out] orientation   Resulting orientation
 */
template <class VectorT>
inline void get_orientation(float roll, float pitch, float yaw, VectorT &orientation)
{
  tf2::Matrix3x3 mat;
  mat.setRPY(roll, pitch, yaw);
  tf2::Vector3 column = mat.getColumn(0);

  orientation << column.getX(), column.getY(), column.getZ();
}


/**
 * \brief  Computes the difference between two poses
 *
 * \param[in]  p1    First pose
 * \param[in]  p2    Second pose
 * \param[out] diff  Difference between p1 and p2
 */
inline void diff_pose(
  const geometry_msgs::Pose &p1,
  const geometry_msgs::Pose &p2,
  geometry_msgs::Pose &diff)
{
  diff.position.x = p1.position.x - p2.position.x;
  diff.position.y = p1.position.y - p2.position.y;
  diff.position.z = p1.position.z - p2.position.z;

  double roll1, pitch1, yaw1;
  double roll2, pitch2, yaw2;
  to_euler(p1.orientation, roll1, pitch1, yaw1);
  to_euler(p2.orientation, roll2, pitch2, yaw2);
  to_quaternion(roll1 - roll2, pitch1 - pitch2, yaw1 - yaw2, diff.orientation);
}


/**
 * \brief  Computes the difference between two poses
 *
 * \param[in]  p1    First pose
 * \param[in]  p2    Second pose
 * \param[out] diff  Difference between p1 and p2
 */
inline void diff_pose(
  const geometry_msgs::Pose &p1,
  const geometry_msgs::Pose &p2,
  mf_common::EulerPose &diff)
{
  diff.x = p1.position.x - p2.position.x;
  diff.y = p1.position.y - p2.position.y;
  diff.z = p1.position.z - p2.position.z;

  double roll1, pitch1, yaw1;
  double roll2, pitch2, yaw2;
  to_euler(p1.orientation, roll1, pitch1, yaw1);
  to_euler(p2.orientation, roll2, pitch2, yaw2);
  diff.roll = roll1 - roll2;
  diff.pitch = pitch1 - pitch2;
  diff.yaw = yaw1 - yaw2;
}

/**
 * \brief  Fills a Eigen DiagonalMatrix from a std::Vector
 *
 * \param[in]  v  Vector containing the diagonal terms of the matrix
 * \param[out] D  Diagonal matrix to fill
 */
template <class T>
inline void fill_diag_mat(
  const std::vector<T> &v,
  Eigen::DiagonalMatrix<T, Eigen::Dynamic> &D)
{
  Eigen::Matrix<T, Eigen::Dynamic, 1> vec(v.size());

  for (int k = 0; k < v.size(); k++)
    vec(k) = v[k];

  D = vec.asDiagonal();
}

/**
 * \brief  Fills a Eigen matrix from a std::Vector
 *
 * \param[in]  v  Vector containing the diagonal terms of the matrix
 * \param[out] D  Square diagonal matrix to fill
 */
template <class T, class MatrixT>
inline void fill_diag_mat(
  const std::vector<T> &v,
  MatrixT &D)
{
  int n = v.size();
  D = MatrixT::Zero(n, n);

  for (int k = 0; k < n; k++) {
    D(k, k) = v[k];
  }
}


}  // namespace mfcpp

#endif
