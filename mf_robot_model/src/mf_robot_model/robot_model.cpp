/**
 * @file
 *
 * \brief  Definition of physical underwater robot model
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "robot_model.hpp"
#include <boost/numeric/odeint.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>

using Eigen::Vector3d;
using Eigen::Matrix3d;

using namespace std;
namespace pl = std::placeholders;


namespace mfcpp {


RobotModel::RobotModel()
{
  g_ = 9.8;
}


RobotModel::RobotModel(const vector<double> &c)
{
  c_ = c;
  g_ = 9.8;
}


RobotModel::~RobotModel()
{

}


void RobotModel::integrate(state_type &state, const input_type &input, double t1,
  double t2, double init_step)
{
  u_ = input;

  size_t steps = boost::numeric::odeint::integrate(
    std::bind(&RobotModel::ode, this, pl::_1, pl::_2, pl::_3),
    state, t1, t2, init_step
  );
}


void RobotModel::ode(const state_type &x, state_type &dxdt, const double t)
{
  Vector3d pos_dt = jac_pos(x[3], x[4], x[5]) * Vector3d(x[6], x[7], x[8]);
  dxdt[0] = pos_dt[0];
  dxdt[1] = pos_dt[1];
  dxdt[2] = pos_dt[2];

  Vector3d orient_dt = jac_orient(x[3], x[4], x[5]) * Vector3d(x[9], x[10], x[11]);
  dxdt[3] = orient_dt[0];
  dxdt[4] = orient_dt[1];
  dxdt[5] = orient_dt[2];

  double phi = x[3];
  double theta = x[4];

  dxdt[6] = c_[1]*x[6] + c_[2]*abs(x[6])*x[6] + c_[3]*abs(u_[0])*u_[0] - x[12]*g_*sin(theta);
  dxdt[7] = x[12]*g_*cos(theta)*sin(phi) + c_[11]*x[7];
  dxdt[8] = x[12]*g_*cos(theta)*cos(phi) + c_[11]*x[8];
  dxdt[9] = 0;
  dxdt[10] = c_[4]*x[10] + c_[5]*abs(x[10])*x[10] + c_[6]*sin(theta) + c_[7]*x[6]*x[6]*u_[2];
  dxdt[11] = c_[8]*x[11] + c_[9]*abs(x[11])*x[11] + c_[10]*x[6]*x[6]*u_[1];
  dxdt[12] = -1/c_[12]*x[12] + u_[3];
}


Matrix3d RobotModel::jac_pos(double phi, double theta, double psi)
{
  Matrix3d J;

  double c_ph = cos(phi);
  double s_ph = sin(phi);
  double c_th = cos(theta);
  double s_th = sin(theta);
  double c_ps = cos(psi);
  double s_ps = sin(psi);

  J << c_ps*c_th, -s_ps*c_th + c_ps*s_th*s_ph,  s_ps*s_th + c_ps*c_ph*s_th,
       s_ps*c_th,  c_ps*c_ph + s_ph*s_th*s_ps, -c_ps*s_th + s_th*s_ps*c_ph,
       -s_th, c_th*s_ph, c_th*c_ph;

  return J;
}


Matrix3d RobotModel::jac_orient(double phi, double theta, double psi)
{
  Matrix3d J;

  double c_ph = cos(phi);
  double s_ph = sin(phi);
  double c_th = cos(theta);
  double t_th = tan(theta);

  J << 1, s_ph*t_th,  c_ph*t_th,
       0, c_ph,      -s_ph,
       0, s_ph/c_th,  c_ph/c_th;

  return J;
}


}  // namespace mfcpp
