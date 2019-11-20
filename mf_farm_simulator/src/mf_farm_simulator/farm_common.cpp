/**
 * @file
 *
 * \brief  Definition of common functions for farm simulator
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "farm_common.hpp"
#include <random>
#include <cstdlib>
#include <iostream>


using namespace std;

namespace mfcpp {


vector<tf2::Vector3> get_alga_coord(const AlgaeLine &line, const Alga &alga)
{
  tf2::Vector3 X1 = line.line.p1;
  tf2::Vector3 X2 = line.line.p2;
  tf2::Vector3 z(0, 0, 1);
  tf2::Vector3 y = (X2-X1) / tf2::tf2Distance(X1, X2);
  tf2::Vector3 x = tf2::tf2Cross(y, z);

  tf2::Vector3 X = alga.position;
  float psi = alga.orientation;
  float H = alga.length;
  float W = alga.width;

  vector<tf2::Vector3> coord(4);
  coord[0] = X - W/2*y;
  coord[1] = X + W/2*y;
  coord[2] = coord[1] - H*(cos(psi)*z - sin(psi)*x);
  coord[3] = coord[0] - H*(cos(psi)*z - sin(psi)*x);
}


ostream &operator<<(ostream &stream, const tf2::Vector3 &v)
{
  stream << "(x=" << v.getX() << " ; y=" << v.getY() << " ; z=" << v.getZ() << ")";
}


/**
 * \brief  Seed initialiser for generation of random numbers
 */
random_device random_device_;

/**
 * \brief  Random generator for generation of random numbers
 */
mt19937 random_generator_(random_device_());


double rand_gaussian(double mu, double sigma)
{
  normal_distribution<double> distribution(mu, sigma);
  return distribution(random_generator_);
}


double rand_uniform(double a, double b)
{
  uniform_real_distribution<double> distribution(a, b);
  return distribution(random_generator_);
}


bool rand_bernoulli(double p)
{
  bernoulli_distribution distribution(p);
  return distribution(random_generator_);
}





}  // namespace mfcpp
