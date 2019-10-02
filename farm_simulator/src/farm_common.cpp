/**
 * @file
 *
 * \brief  Definition of common functions for farm simulator
 * \author Corentin Chauvin-Hamea
 * \date   2019
 */

#include "farm_common.hpp"
#include <random>
#include <cstdlib>
#include <iostream>


namespace mfcpp {

  std::ostream &operator<<(std::ostream &stream, const tf2::Vector3 &v)
  {
    stream << "(x=" << v.getX() << " ; y=" << v.getY() << " ; z=" << v.getZ() << ")";
  }


  /**
   * \brief  Seed initialiser for generation of random numbers
   */
  std::random_device random_device_;

  /**
   * \brief  Random generator for generation of random numbers
   */
  std::mt19937 random_generator_(random_device_());


  double rand_gaussian(double mu, double sigma)
  {
    std::normal_distribution<double> distribution(mu, sigma);
    return distribution(random_generator_);
  }


  double rand_uniform(double a, double b)
  {
    std::uniform_real_distribution<double> distribution(a, b);
    return distribution(random_generator_);
  }





}  // namespace mfcpp
