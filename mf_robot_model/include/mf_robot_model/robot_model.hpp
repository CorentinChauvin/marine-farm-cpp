/**
 * @file
 *
 * \brief  Declaration of physical underwater robot model
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#ifndef ROBOT_MODEL_HPP
#define ROBOT_MODEL_HPP

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
     * \brief  Integrates the model over a period of time
     *
     * It will update the state by integrating the dynamic model of the robot.
     * It assumes that the inputs are constant during the integration.
     *
     * \note  It is based on the error-controlled `runge_kutta54_cash_karp`
     *        stepper (5th order) and uses adaptive step-size.
     *
     * \param state       Initial state to update
     * \param input       Input to the actuators
     * \param t1          Starting time of the integration
     * \param t2          Ending time of the integration
     * \param init_step   Initial step size for the integration
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
     * \param[in]  x_0  Nominal state
     * \param[in]  u_0  Nominal input
     * \param[out] A    A matrix
     * \param[out] B    B matrix
     */
    void get_linear_matrices(const state_type &x_0, const input_type &u_0,
      Eigen::MatrixXd &A, Eigen::MatrixXd &B);

      /**
       * \brief  Gets the matrices of the discretised linearised system
       *
       * The system \f$ \dot{\Delta x} = A \Delta x + B \Delta u \f$ is discretised
       * by \f$ A_d = e^{dt.A} \f$ and \f$ B_d = (\int_{0}^{dt} e^{s.A} ds) B \f$.
       * The integral is approximated by a Riemann sum.
       *
       * \param[in]  x_0  Nominal state
       * \param[in]  u_0  Nominal input
       * \param[in]  dt   Discretisation interval
       * \param[out] Ad   Discretised A matrix
       * \param[out] Bd   Discretised B matrix
       * \param[in]  N    Number of terms in the Riemann sum
       */
    void get_lin_discr_matrices(const state_type &x_0, const input_type &u_0,
      float dt, Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd, int N=10);

    /**
     * \brief  Computes the propeller speed in steady state
     *
     * \note  This assumes that delta_m = 0
     * \param speed  Desired steady state speed
     */
    double inline steady_propeller_speed(double speed);

    /**
     * \brief  Computes the horizontal speed in steady state
     *
     * \note  This assumes that the propeller speed is constant and the robot is
     *        horizontal
     * \param n  Rotational speed of the propeller
     */
    double inline steady_speed(double n);

    /**
     * \brief  Computes the lateral turning radius
     *
     * \param u        Speed of the robot
     * \param delta_r  Angle of the lateral rudder
     */
    double inline lat_turn_radius(double u, double delta_r);

    /**
     * \brief  Computes the elevation turning radius
     *
     * \param u        Speed of the robot
     * \param delta_e  Angle of the elevation rudder
     */
    double inline elev_turn_radius(double u, double delta_e);

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
     * \note  ODE stands for Ordinary Differential Equation!
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
    inline double sign(double x);

};


double inline RobotModel::steady_propeller_speed(double speed)
{
  double a = 1/c_[3] * (-c_[1]*speed - c_[2]*speed*speed);
  return sign(speed) * sqrt(abs(a));
}


inline double RobotModel::steady_speed(double propeller_speed)
{
  double delta = pow(c_[1], 2) - 4*c_[2]*c_[3]*pow(propeller_speed, 2);
  return -(c_[1] + sqrt(delta)) / (2*c_[2]);
}


double inline RobotModel::lat_turn_radius(double u, double delta_r)
{
  double K = -4 * c_[9] * c_[10] * delta_r;
  double r = (-c_[8] - sqrt(pow(c_[8], 2) + K*pow(u, 2))) / (2*c_[9]);
  double R_squared = -c_[10]*delta_r / (c_[8]/r + c_[9]);

  return sqrt(R_squared);
}


double inline RobotModel::elev_turn_radius(double u, double delta_e)
{
  double K = -4 * c_[5] * c_[7] * delta_e;
  double r = (-c_[4] - sqrt(pow(c_[4], 2) + K*pow(u, 2))) / (2*c_[5]);
  double R_squared = -c_[7]*delta_e / (c_[4]/r + c_[5]);

  return sqrt(R_squared);
}


inline double RobotModel::sign(double x)
{
  return (double) (x >= 0);
}




}  // namespace mfcpp


#endif
