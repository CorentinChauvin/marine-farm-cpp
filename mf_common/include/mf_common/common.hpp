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
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Matrix3x3.h>
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
