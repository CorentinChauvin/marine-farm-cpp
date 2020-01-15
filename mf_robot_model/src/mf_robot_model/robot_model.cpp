/**
 * @file
 *
 * \brief  Definition of physical underwater robot model
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "robot_model.hpp"
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <boost/numeric/odeint.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;

using namespace std;
namespace pl = std::placeholders;


namespace mfcpp {


RobotModel::RobotModel()
{
  g_ = 9.81;
}


RobotModel::RobotModel(const vector<double> &c)
{
  c_ = c;
  g_ = 9.81;
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


void RobotModel::get_lin_matrices(const state_type &x_0, const input_type &u_0,
  Eigen::MatrixXd &A, Eigen::MatrixXd &B)
{
  const state_type &x = x_0;  // for convenience
  const input_type &u = u_0;
  A = MatrixXd::Zero(x.size(), x.size());
  B = MatrixXd::Zero(u.size(), u.size());

  double c3 = cos(x[3]);  double s3 = sin(x[3]);
  double c4 = cos(x[4]);  double s4 = sin(x[4]);  double t4 = tan(x[4]);
  double c5 = cos(x[5]);  double s5 = sin(x[5]);

  // A matrix
  A(0, 3) = (s5*s3 + c5*s4*c3)*x[7] + (s5*c3 - c5*s3*s4)*x[8];
  A(0, 4) = -c5*s4*x[6] + c5*c4*s3*x[7] + c5*c3*c4*x[8];
  A(0, 5) = -s5*c4*x[6] +(-c5*c3 - s5*s4*s3)*x[7] + (c5*s3 - s5*c3*s4)*x[8];
  A(0, 6) = c5*c4;
  A(0, 7) = -s5*c3 + c5*s4*s3;
  A(0, 8) = s5*s3 + c5*c3*s4;

  A(1, 3) = (-c5*s3 + s5*s4*c3)*x[7] + (-c5*c3 - s5*s3*s4)*x[8];
  A(1, 4) = -s5*s4*x[6] + s5*c4*s3*x[7] + s5*c3*c4*x[8];
  A(1, 5) = c5*c4*x[6] + (-s5*s3 + c5*s4*s3)*x[7] + (s5*s3 + c5*c3*s4)*x[8];
  A(1, 6) = s5*c4;
  A(1, 7) = c5*c3 + s5*s4*s3;
  A(1, 8) = -c5*s3 + s5*s4*c3;

  A(2, 3) = c4*c3*x[7] - c4*s3*x[8];
  A(2, 4) = -c4*x[6] - s4*s3*x[7] -s4*c3*x[8];
  A(2, 6) = -s4;
  A(2, 7) = c4*s3;
  A(2, 8) = c4*c3;

  A(3, 3) = c3*t4*x[10] - s3*t4*x[11];
  A(3, 4) = (s3*x[10] + c3*x[11]) * (1 + t4*t4);
  A(3, 9) = 1;
  A(3, 10) = s3*t4;
  A(3, 11) = c3*t4;

  A(4, 3) = -s3*x[10] - c3*x[11];
  A(4, 10) = c3;
  A(4, 11) = -s3;

  A(5, 3) = (c3*x[10] - s3*x[11]) / c4;
  A(5, 4) = s4 / (c4*c4) * (s3*x[10] + c3*x[11]);
  A(5, 10) = s3/c4;
  A(5, 11) = c3/c4;

  A(6, 4) = -1/c_[12]*x[12]*c4;
  A(6, 6) = c_[1] + 2*sign(x[6])*x[6];
  A(6, 12) = -1/c_[12]*s4;

  A(7, 3) = g_*x[12]*c4*c3;
  A(7, 4) = -g_*x[12]*s4*s3;
  A(7, 7) = c_[11];
  A(7, 12) = g_*c4*s3;

  A(8, 3) = -g_*x[12]*c4*s3;
  A(8, 4) = -g_*x[12]*s4*c3;
  A(8, 8) = c_[11];
  A(8, 12) = g_*c4*c3;

  A(10, 4) = c_[6]*c4;
  A(10, 10) = c_[4] + 2*c_[5]*sign(x[10])*x[10];

  A(11, 11) = c_[8] + 2*c_[9]*sign(x[11])*x[11];

  A(12, 12) = -1/c_[12];

  // B matrix
  B(6, 0) = 2*c_[3]*sign(u[0])*u[0];
  B(10, 2) = c_[7]*x[6]*x[6];
  B(11, 1) = c_[10]*x[6]*x[6];
  B(12, 3) = 1;
}


void RobotModel::get_lin_discr_matrices(const state_type &x_0, const input_type &u_0,
  Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd, float dt, int N)
{
  Eigen::MatrixXd A, B;
  get_lin_matrices(x_0, u_0, A, B);

  Ad = (dt*A).exp();

  MatrixXd sum = MatrixXd::Zero(u_0.size(), u_0.size());
  float ds = dt / N;

  for (int k = 0; k < N; k++) {
    sum += ds * (k*ds*A).exp();
  }
  Bd = sum * B;
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
