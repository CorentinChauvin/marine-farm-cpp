/**
 * @file
 *
 * \brief  Definition of 2D perlin noise generator
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#include "perlin_noise.hpp"
#include <random>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>

namespace mfcpp {


PerlinNoiseGenerator::Vec2d::Vec2d():
   x(0), y(0)
{

}


PerlinNoiseGenerator::Vec2d::Vec2d(float _x, float _y):
   x(_x), y(_y)
{

}


void PerlinNoiseGenerator::Vec2d::operator=(const Vec2d &a)
{
  x = a.x;
  y = a.y;
}


float PerlinNoiseGenerator::Vec2d::norm2()
{
  return x*x + y*y;
}


float PerlinNoiseGenerator::Vec2d::norm()
{
  return std::sqrt(x*x + y*y);
}


/*
 * Function definitions of PerlinNoiseGenerator
 */

PerlinNoiseGenerator::PerlinNoiseGenerator()
{
  height_ = 1;
  width_ = 1;
  n_height_ = 1;
  n_width_ = 1;
}


PerlinNoiseGenerator::PerlinNoiseGenerator(float height, float width,
  unsigned int n_height, unsigned int n_width)
{
  height_ = height;
  width_ = width;
  n_height_ = n_height;
  n_width_ = n_width;
  gradients_.resize(n_height+1, std::vector<Vec2d>(n_width+1, Vec2d()));
}


PerlinNoiseGenerator::~PerlinNoiseGenerator()
{

}


void PerlinNoiseGenerator::configure(float height, float width,
  unsigned int n_height, unsigned int n_width)
{
  height_ = height;
  width_ = width;
  n_height_ = n_height;
  n_width_ = n_width;
  gradients_.resize(n_height+1, std::vector<Vec2d>(n_width+1, Vec2d()));
}


void PerlinNoiseGenerator::randomise_gradients()
{
  std::mt19937 random_generator(random_device_());
  std::uniform_real_distribution<double> distribution(-1, 1);
  hash_gradients_.resize(256, Vec2d());

  for (int i = 0; i < 256; i++) {
    Vec2d v;

    do {
      v.x = distribution(random_generator);
      v.y = distribution(random_generator);
    } while (v.norm2() > 1.0 && v.norm2() == 0.0);

    float norm = v.norm();
    v.x /= norm;
    v.y /= norm;

    hash_gradients_[i] = v;
  }
}


void PerlinNoiseGenerator::generate()
{
  // Generate permutations
  std::vector<unsigned int> permut(256);
  for (int i = 0; i < 256; i++) {
    permut[i] = i;
  }
  std::random_shuffle (permut.begin(), permut.end());

  // Assign gradients to each node
  for (unsigned int i = 0; i <= n_height_; i++) {
    for (unsigned int j = 0; j <= n_width_; j++) {
      gradients_[i][j] = hash_gradients_[permut[(i + permut[j]) & 255]];
    }
  }
}


double PerlinNoiseGenerator::evaluate(float x, float y) const
{
  if (x < 0 || y < 0 || x > height_ || y > width_) {
    std::cout << "[PerlinNoiseGenerator] Error: specified coordinates not valid\n";
  }

  // Find the cell corresponding to the coordinates
  unsigned int i0 = x / height_ * n_height_;
  unsigned int j0 = y / width_ * n_width_;
  unsigned int i1 = i0 + 1;
  unsigned int j1 = j0 + 1;
  float x0 = float(i0)/float(n_height_)*height_;
  float y0 = float(j0)/float(n_width_)*width_;

  // Compute the dot products between the distance to the nodes and the gradients
  // d0 | d2   | -> y/j
  // -------   V
  // d1 | d3   x/i
  float d0 = dot_dist_grad(i0, j0, x, y);
  float d1 = dot_dist_grad(i1, j0, x, y);
  float d2 = dot_dist_grad(i0, j1, x, y);
  float d3 = dot_dist_grad(i1, j1, x, y);

  // Interpolate the dot products
  float t_x = (x - x0) / (height_/n_height_);
  float t_y = (y - y0) / (width_/n_width_);

  float int1 = interpolate(d0, d1, fade(t_x));
  float int2 = interpolate(d2, d3, fade(t_x));
  float output = interpolate(int1, int2, fade(t_y));

  // Bring output from [-1, 1] to [0, 1]
  if (output < -1) output = -1;
  if (output > 1) output = 1;
  output = output/2 + 0.5;

  return output;
}


float PerlinNoiseGenerator::dot_product(Vec2d u, Vec2d v) const
{
  return u.x*v.x + u.y*v.y;
}


float PerlinNoiseGenerator::dot_dist_grad(unsigned int i, unsigned int j,
  float x, float y) const
{
  float x_node = float(i) / float(n_height_) * height_;
  float y_node = float(j) / float(n_width_) * width_;

  return dot_product(Vec2d(x-x_node, y-y_node), gradients_[i][j]);
}


float PerlinNoiseGenerator::interpolate(float a, float b, float t) const
{
  return (1-t)*a + t*b;
}


}  // namespace mfcpp
