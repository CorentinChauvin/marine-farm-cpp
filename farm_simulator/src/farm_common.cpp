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


namespace mfcpp {

  /**
   * \brief  Seed initialiser for generation of random numbers
   */
  std::random_device random_device_;

  /**
   * \brief  Random generator for generation of random numbers
   */
  std::mt19937 random_generator_(random_device_());


  double random_gaussian(double mu, double sigma)
  {
    std::normal_distribution<double> distribution(mu, sigma);
    return distribution(random_generator_);
  }


  double random_uniform(double a, double b)
  {
    std::uniform_real_distribution<double> distribution(a, b);
    return distribution(random_generator_);
  }





}  // namespace mfcpp
