/**
 * @file
 *
 * \brief  Declaration of a class for 3D spline interpolation
 * \author Corentin Chauvin-Hameau
 * \date   2020
 */

#ifndef SPLINE_HPP
#define SPLINE_HPP

#include <eigen3/Eigen/Dense>
#include <vector>

namespace mfcpp {

/**
 * \brief  Class for spline interpolation
 *
 * Interpolates a list of given positions and orientations by cubic splines at
 * constant speed. The evaluation function is optimised for successive calls
 * with increasing time.
 */
class Spline {
  public:
    /// \brief  Default constructor
    Spline();

    /**
     * \brief  Constructor for a spline given by a list of poses
     *
     * Will call `Spline::prepare()`.
     *
     * \param positions       Positions to interpolate
     * \param orientations    Corresponding orientations
     * \param speed           Desired constant speed
     */
    Spline(
      const std::vector<Eigen::Vector3f> &positions,
      const std::vector<Eigen::Vector3f> &orientations,
      float speed
    );

    /**
     * \brief  Sets the poses to interpolate
     *
     * \param positions     Positions to interpolate
     * \param orientations  Corresponding orientations
     */
    void set_poses(
      const std::vector<Eigen::Vector3f> &positions,
      const std::vector<Eigen::Vector3f> &orientations
    );

    /**
     * \brief  Set the desired speed on the path
     *
     * \param speed  Desired speed
     */
    void set_speed(float speed);

    /**
     * \brief  Evaluates the spline at a specific time instant (assuming constant speed)
     *
     * If t is larger than the time t_max when the last specified pose is reached,
     * the output pose will be this last pose.
     *
     * \param[in]  t             Time instant (should be positive)
     * \param[out] position      Interpolated position
     * \param[out] orientation   Interpolated orientation
     * \param[out] last_reached  Whether the last pose has been reached
     */
    void evaluate(
      float t,
      Eigen::Vector3f &position,
      Eigen::Vector3f &orientation,
      bool &last_reached
    );


  private:
    int n_;  ///<  Number of poses to interpolate
    float speed_;  ///<  Constant speed to adopt on the path
    std::vector<Eigen::Vector3f> p_;  ///<  Positions to interpolate
    std::vector<Eigen::Vector3f> o_;  ///<  Corresponding orientations
    std::vector<std::vector<Eigen::Vector3f>> a_;  ///<  Spline parameters for each segment (ie between each pose)
    bool prepared_;    ///<  Whether the class is ready for interpolation
    float last_s_;  ///<  Curvilinear abscissa of the last evaluated point
    float last_t_;  ///<  Time instant of the last evaluated point

    /**
     * \brief  Computes the spline parameters
     */
    void compute_parameters();

    /**
     * \brief  Prepares the class for interpolation
     *
     * Computes the spline parameters and the time instants at the given points.
     */
    void prepare();

    /**
     * \brief  Evaluates the spline at a specific curvilinear abscissa
     *
     * \param s  Curvilinear abscissa (in [0, n])
     * \return  Interpolated position
     */
    Eigen::Vector3f evaluate_position(float s);

    /**
     * \brief  Computes the orientation at a specific curvilinear abscissa
     *
     * \param s  Curvilinear abscissa (in [0, n])
     * \return  Interpolated orientation
     */
    Eigen::Vector3f compute_orientation(float s);

    /**
     * \brief  Computes the derivative of the curvilinear abscissa wrt time
     *
     * This is the ODE used to compute the abscissa corresponding to at time.
     *
     * \param[in]   Abscissa at which to compute the derivative (vector of size 1)
     * \param[out]  Derivative of the abscissa (vector of size 1)
     * \param[in]   Time parameter (not used here since no time dependency)
     */
    void deriv_abscissa(
      const std::vector<double> &s,
      std::vector<double> &dsdt,
      const double t
    );

    /**
     * \brief  Computes the curvilinear abscissa corresponding to a time instant
     *
     * The function is optimised for successive calls with an increasing s.
     *
     * \param t  Time at which to interpolate (should be positive)
     * \return  Corresponding curvilinear abscissa
     */
    float compute_abscissa(float t);
};


}  // namespace mfcpp

#endif
