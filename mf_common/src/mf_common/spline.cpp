/**
 * @file
 *
 * \brief  Definition of a class for 3D spline interpolation
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#include "spline.hpp"
#include <eigen3/Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include <iostream>

using namespace std;
using Eigen::Vector3f;
namespace pl = std::placeholders;


namespace mfcpp {

Spline::Spline():
  prepared_(false)
{

}


Spline::Spline(
  const std::vector<Eigen::Vector3f> &positions,
  const std::vector<Eigen::Vector3f> &orientations,
  float speed)
{
  set_poses(positions, orientations);
  set_speed(speed);
  prepare();
}


void Spline::set_poses(
  const std::vector<Eigen::Vector3f> &positions,
  const std::vector<Eigen::Vector3f> &orientations)
{
  if (positions.size() != orientations.size())
    cout << "[Spline] Not the same number of specified positions and orientations" << endl;
  if (positions.size() == 0)
    cout << "[Spline] No specified position" << endl;

  p_ = positions;
  o_ = orientations;
  n_ = positions.size() - 1;
  prepared_ = false;
}


void Spline::set_speed(float speed)
{
  speed_ = speed;
  prepared_ = false;
}


void Spline::evaluate(
  float t,
  Eigen::Vector3f &position,
  Eigen::Vector3f &orientation,
  bool &last_reached)
{
  if (!prepared_)
    prepare();

  if (t < 0) {
    cout << "[Spline] Evaluating at t<0 (t=" << t << ")" << endl;
    return;
  }

  float s = compute_abscissa(t);

  if (s >= n_) {
    last_reached = true;
    s = n_;
  } else
    last_reached = false;

  position = evaluate_position(s);
  orientation = compute_orientation(s);
}


void Spline::prepare()
{
  compute_parameters();

  last_s_ = 0;
  last_t_ = 0;

  prepared_ = true;
}


void Spline::compute_parameters()
{
  a_.resize(n_, vector<Vector3f>(4));

  for (int k = 0; k < n_; k++) {
    a_[k][0] = p_[k];
    a_[k][1] = o_[k];
    a_[k][2] = 3*(p_[k+1] - p_[k]) - 2*o_[k] - o_[k+1];
    a_[k][3] = 2*(p_[k] - p_[k+1]) + o_[k] + o_[k+1];
  }
}


Eigen::Vector3f Spline::evaluate_position(float s)
{
  if (s < 0)
    return p_[0];
  if (s >= n_)
    return p_[n_];

  int k = (int) s;

  return a_[k][0] + (s-k)*a_[k][1] + pow(s-k, 2)*a_[k][2] + pow(s-k, 3)*a_[k][3];
}


Eigen::Vector3f Spline::compute_orientation(float s)
{
  if (s < 0)
    return o_[0];
  if (s >= n_)
    return o_[n_];

  int k = (int) s;

  Vector3f orientation = a_[k][1] + 2*(s-k)*a_[k][2] + 3*pow(s-k, 2)*a_[k][3];
  return orientation;
}


void Spline::deriv_abscissa(
  const std::vector<double> &s,
  std::vector<double> &dsdt,
  const double t)
{
  dsdt[0] =  speed_ / compute_orientation(s[0]).norm();
}


float Spline::compute_abscissa(float t)
{
  float s0 = last_s_;
  float t0 = last_t_;
  vector<double> state(1, s0);

  if (t0 != t) {
    size_t steps = boost::numeric::odeint::integrate(
      std::bind(&Spline::deriv_abscissa, this, pl::_1, pl::_2, pl::_3),
      state, t0, t, float(t-t0)/2
    );
  }

  last_s_ = state[0];
  last_t_ = t;

  return state[0];
}


}  // namespace mfcpp
