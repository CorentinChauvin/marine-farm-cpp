/**
 * @file
 *
 * \brief  Declaration of physical underwater robot model
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#ifndef ROBOT_MODEL_HPP
#define ROBOT_MODEL_HPP

#include <eigen3/Eigen/Dense>
#include <vector>


namespace mfcpp {


/**
 * \brief  Class defining the robot model
 *
 * The state is as follow:
 * - \f$ x_0 = X      \f$
 * - \f$ x_1 = Y      \f$
 * - \f$ x_2 = Z      \f$
 * - \f$ x_3 = V      \f$
 * - \f$ x_4 = \gamma \f$
 * - \f$ x_5 = \chi   \f$
 * - \f$ x_6 = \alpha \f$
 * - \f$ x_7 = \mu    \f$
 * - \f$ x_8 = T      \f$
 * - \f$ x_9 = M      \f$
 *
 * The inputs are as follow:
 * - TODO
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

  private:
    /// \brief  Model constants
    /// \warning  c[0] is not used to respect notation
    std::vector<double> c_;

    /// Gravity
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

};

}  // namespace mfcpp



#endif
