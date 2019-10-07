/**
 * @file
 *
 * \brief  Declaration of 2D perlin noise generator
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#ifndef PERLIN_NOISE_HPP
#define PERLIN_NOISE_HPP

#include <random>
#include <vector>
#include <algorithm>
#include <iostream>

namespace mfcpp {
  /**
   * \brief  Perlin noise generator
   */
  class PerlinNoiseGenerator {
    public:
      /**
       * \brief  Default constructor
       */
      PerlinNoiseGenerator();

      /**
       * \brief  Constructor initialising the generator with the given parameters
       *
       * \param height    Height of the 2D space in which to generate the noise
       * \param width     Width of the 2D space in which to generate the noise
       * \param n_height  Size of the discretised grid in the first dimension
       * \param n_width   Size of the discretised grid in the second dimensio
       */
      PerlinNoiseGenerator(float height, float width, unsigned int n_height,
        unsigned int n_width);

      ~PerlinNoiseGenerator();

      /**
       * \brief  Configures the generator
       *
       * \param height    Height of the 2D space in which to generate the noise
       * \param width     Width of the 2D space in which to generate the noise
       * \param n_height  Size of the discretised grid in the first dimension
       * \param n_width   Size of the discretised grid in the second dimension
       */
      void configure(float height, float width, unsigned int n_height,
        unsigned int n_width);

      /**
       * \brief  Fills the hash list with random gradients
       */
      void randomise_gradients();

      /**
       * \brief  Populates the grid with random gradients from the hash list
       */
      void generate();

      /**
       * \brief  Evaluates the perlin noise at a given position
       *
       * \param x  First coordinate where to evaluate
       * \param y  Second coordinate where to evaluate
       * \return  Perlin noise value at (x, y)
       */
      double evaluate(float x, float y) const;

      /**
       * \brief  Applies a polynomial to accentuate the given value
       *
       * Values under 0.5 are pushed towards 0, while values above 0.5 are
       * pushed towards 1.
       *
       * \param x  Given value
       * \return  Accentuated value
       */
      inline float accentuate(float x) const;

    private:
      /**
       * \brief  Two dimensional vector
       */
      struct Vec2d {
        float x;  ///<  First coordinate
        float y;  ///<  Second coordinate

        Vec2d();
        Vec2d(float _x, float _y);
        void operator=(const Vec2d &a);

        /**
         * \brief  Computes the square of the euclidean norm of the vector
         * \return  Square of the euclidean norm of the vector
         */
        float norm2();

        /**
         * \brief  Computes the euclidean norm of the vector
         * \return  Euclidean norm of the vector
         */
        float norm();
      };

      /// Height of the 2D space in which to generate the noise
      float height_;

      /// Width of the 2D space in which to generate the noise
      float width_;

      /// Size of the discretised grid in the first dimension
      unsigned int n_height_;

      /// Size of the discretised grid in the second dimension
      unsigned int n_width_;

      /// Gradients evaluated at nodes of the grid
      std::vector<std::vector<Vec2d>> gradients_;

      /// Hash list of random gradients
      std::vector<Vec2d> hash_gradients_;

      ///  Seed initialiser for generation of random numbers
      std::random_device random_device_;

      /**
       * \brief  Computes the dot product between two vectors
       *
       * \param u  First vector
       * \param v  Second vector
       * \return  Dot product between u and v
       */
      float dot_product(Vec2d u, Vec2d v) const;

      /**
       * \brief  Compute the dot product of the distance vector to the node an
       *         the gradient at this node
       *
       * \param i  Index of the node for the first dimension
       * \param j  Index of the node for the second dimension
       * \param x  First coordinate of the evaluated point
       * \param y  Second coordinate of the evaluated point
       */
      float dot_dist_grad(unsigned int i, unsigned int j, float x, float y) const;

      /**
       * \brief  Linear interpolation between two values
       *
       * \param a  First value to interpolate
       * \param b  Second value to interpolate
       * \param t  Interpolation ratio
       * \return  Interpolation between a and b
       */
      float interpolate(float a, float b, float t) const;


      /**
       * \brief  Sixth order polynomial to smooth the noise
       *
       * \param t  Number between 0 and 1
       * \return  6t^5 - 15t^4 + 10t^3
       */
      inline float fade(float t) const;
  };


  /*
   * Defintion of inline functions
   */
  inline float PerlinNoiseGenerator::fade(float t) const
  {
    return t * t * t * (t * (t * 6 - 15) + 10);  // 6t^5 - 15t^4 + 10t^3
  }


  inline float PerlinNoiseGenerator::accentuate(float x) const
  {
    return fade(fade(fade(fade(x))));
  }

}

#endif
