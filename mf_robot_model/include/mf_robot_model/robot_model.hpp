/**
 * @file
 *
 * \brief  Declaration of physical underwater robot model
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#ifndef ROBOT_MODEL_HPP
#define ROBOT_MODEL_HPP

#include "mf_common/common.hpp"
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>


namespace mfcpp {


/**
 * \brief  Class defining the robot model
 *
 * The state is as follow:
 * - \f$ x_0 = x         \f$
 * - \f$ x_1 = y         \f$
 * - \f$ x_2 = z         \f$
 * - \f$ x_3 = \Phi      \f$
 * - \f$ x_4 = \theta    \f$
 * - \f$ x_5 = \psi      \f$
 * - \f$ x_6 = u         \f$
 * - \f$ x_7 = v         \f$
 * - \f$ x_8 = w         \f$
 * - \f$ x_9 = p         \f$
 * - \f$ x_{10} = q        \f$
 * - \f$ x_{11} = r        \f$
 * - \f$ x_{12} = \Delta m \f$
 *
 * The inputs are as follow:
 * - \f$ u_0 = n        \f$
 * - \f$ u_1 = \delta_r \f$
 * - \f$ u_2 = \delta_e \f$
 * - \f$ u_3 = P        \f$
 */
class RobotModel
{
  public:
    /// Data type of the state
    typedef std::vector<double> state_type;

    /// Data type of the input
    typedef std::vector<double> input_type;

    RobotModel();
    RobotModel(const std::vector<double> &c);
    ~RobotModel();


    /**
     * \brief  Evaluate the ODE
     *
     * Public version of the ODE
     *
     * \param x     Current state
     * \param u     Current input
     * \param dxdt  Current time derivative of the state
     * \param t     Current time
     */
    void eval_ode(const state_type &x, const input_type &u, state_type &dxdt, const double t);

    /**
     * \brief  Integrates the model over a period of time
     *
     * It will update the state by integrating the dynamic model of the robot.
     * It assumes that the inputs are constant during the integration.
     *
     * It is based on the error-controlled `runge_kutta54_cash_karp` stepper
     * (5th order) and uses adaptive step-size.
     *
     * \param[in,out] state       Initial state to update
     * \param[in]     input       Input to the actuators
     * \param[in]     t1          Starting time of the integration
     * \param[in]     t2          Ending time of the integration
     * \param[in]     init_step   Initial step size for the integration
     */
    void integrate(state_type &state, const input_type &input, double t1,
      double t2, double init_step);

    /**
     * \brief  Gets the matrices of the linearised system
     *
     * If the system is linearised around \f$ (x_0, u_0) \f$, the system can be
     * expressed for new variables \f$ (\Delta x, \Delta u) = (x-x_0, u-u_0) \f$.
     * The ODE then becomes: \f$ \dot{\Delta x} = A \Delta x + B \Delta u \f$.
     *
     * `MatrixT` can either be `Eigen::MatrixXd` or `Eigen::MatrixXf`
     *
     * \param[in]  x_0  Nominal state
     * \param[in]  u_0  Nominal input
     * \param[out] A    A matrix
     * \param[out] B    B matrix
     */
    template <class MatrixT>
    void get_lin_matrices(const state_type &x_0, const input_type &u_0,
      MatrixT &A, MatrixT &B) const;

      /**
       * \brief  Gets the matrices of the discretised linearised system
       *
       * The system \f$ \dot{\Delta x} = A \Delta x + B \Delta u \f$ is discretised
       * by \f$ A_d = e^{dt.A} \f$ and \f$ B_d = (\int_{0}^{dt} e^{s.A} ds) B \f$.
       *
       * \todo  Update how to get B_d
       *
       * `MatrixT` can either be `Eigen::MatrixXd` or `Eigen::MatrixXf`
       *
       * \param[in]  x_0  Nominal state
       * \param[in]  u_0  Nominal input
       * \param[out] Ad   Discretised A matrix
       * \param[out] Bd   Discretised B matrix
       * \param[in]  dt   Discretisation interval
       */
    template <class MatrixT>
    void get_lin_discr_matrices(const state_type &x_0, const input_type &u_0,
      MatrixT &Ad, MatrixT &Bd, float dt) const;

    /**
     * \brief  Overloads get_lin_discr_matrices for Eigen vectors
     *
     * `VectorT` can either be `Eigen::VectorXd` or `Eigen::VectorXf`
     * `MatrixT` can either be `Eigen::MatrixXd` or `Eigen::MatrixXf`
     */
    template <class VectorT, class MatrixT>
    void get_lin_discr_matrices(const VectorT &x_0, const VectorT &u_0,
        MatrixT &Ad, MatrixT &Bd, float dt) const;

    /**
     * \brief  Computes the propeller speed in steady state
     *
     * \note  This assumes that delta_m = 0
     * \param speed  Desired steady state speed
     */
    double inline steady_propeller_speed(double speed) const;

    /**
     * \brief  Computes the horizontal speed in steady state
     *
     * \note  This assumes that the propeller speed is constant and the robot is
     *        horizontal
     * \param n  Rotational speed of the propeller
     */
    double inline steady_speed(double n) const;

    /**
     * \brief  Computes the lateral turning radius
     *
     * \param u        Speed of the robot
     * \param delta_r  Angle of the lateral rudder
     */
    double inline lat_turn_radius(double u, double delta_r) const;

    /**
     * \brief  Computes the elevation turning radius
     *
     * \param u        Speed of the robot
     * \param delta_e  Angle of the elevation rudder
     */
    double inline elev_turn_radius(double u, double delta_e) const;

  private:
    /// \brief  Model constants
    /// \warning  c[0] is not used to respect notation
    std::vector<double> c_;

    /// Gravity acceleration
    double g_;

    /// Model inputs
    input_type u_;

    /**
     * \brief  ODE used to integrate the model
     *
     * \param x     Current state
     * \param dxdt  Current time derivative of the state
     * \param t     Current time
     */
    void ode(const state_type &x, state_type &dxdt, const double t);

    /**
     * \brief  Computes the Jacobian of the position
     *
     * Transforms linear velocities in the global frame to the intertial
     * frame.
     *
     * \param phi     First angle of the robot's orientation
     * \param theta   Second angle of the robot's orientation
     * \param psi     Third angle of the robot's orientation
     * \return  Jacobian of the position
     */
    Eigen::Matrix3d jac_pos(double phi, double theta, double psi);

    /**
     * \brief  Computes the Jacobian of the orientation
     *
     * Transforms angular velocities in the global frame to the inertial frame.
     *
     * \param phi     First angle of the robot's orientation
     * \param theta   Second angle of the robot's orientation
     * \param psi     Third angle of the robot's orientation
     * \return  Jacobian of the orientation
     */
    Eigen::Matrix3d jac_orient(double phi, double theta, double psi);

    /**
     * \brief  Returns the sign of a real number
     *
     * \return 1 if x>=0, -1 otherwise
     */
    inline double sign(double x) const;

};


double inline RobotModel::steady_propeller_speed(double speed) const
{
  double a = 1/c_[3] * (-c_[1]*speed - c_[2]*speed*speed);
  return sign(speed) * sqrt(abs(a));
}


inline double RobotModel::steady_speed(double propeller_speed) const
{
  double delta = pow(c_[1], 2) - 4*c_[2]*c_[3]*pow(propeller_speed, 2);
  return -(c_[1] + sqrt(delta)) / (2*c_[2]);
}


double inline RobotModel::lat_turn_radius(double u, double delta_r) const
{
  double K = -4 * c_[9] * c_[10] * delta_r;
  double r = (-c_[8] - sqrt(pow(c_[8], 2) + K*pow(u, 2))) / (2*c_[9]);
  double R_squared = -c_[10]*delta_r / (c_[8]/r + c_[9]);

  return sqrt(R_squared);
}


double inline RobotModel::elev_turn_radius(double u, double delta_e) const
{
  double K = -4 * c_[5] * c_[7] * delta_e;
  double r = (-c_[4] - sqrt(pow(c_[4], 2) + K*pow(u, 2))) / (2*c_[5]);
  double R_squared = -c_[7]*delta_e / (c_[4]/r + c_[5]);

  return sqrt(R_squared);
}


inline double RobotModel::sign(double x) const
{
  return (double) (x >= 0);
}

/**
 * \brief  Converts a pose into a state message
 *
 * The first 6 components will be filled with the pose (x, y, z, roll, pitch,
 * yaw). The other components of the state will be filled with zeros.
 *
 * \param[in] pose        Pose to convert
 * \param[in] state_size  Size of the state
 * \return  Converted state
 */
inline RobotModel::state_type to_state(
  const geometry_msgs::Pose &pose,
  int state_size)
{
  RobotModel::state_type state(state_size, 0);

  state[0] = pose.position.x;
  state[1] = pose.position.y;
  state[2] = pose.position.z;
  to_euler(pose.orientation, state[3], state[4], state[5]);

  return state;
}


}  // namespace mfcpp


#endif
