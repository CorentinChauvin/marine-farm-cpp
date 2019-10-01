/**
 * @file
 *
 * \brief  Declaration of common structures and functions for farm simulator
 * \author Corentin Chauvin-Hamea
 * \date   2019
 */

#ifndef FARM_COMMON_HPP
#define FARM_COMMON_HPP

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <vector>

namespace mfcpp {
  /**
   * \brief  Rope tensed between two 3D points
   */
  struct Rope
  {
    tf2::Vector3 p1;	///<  Position of the first extremity of the rope
    tf2::Vector3 p2;	///<  Position of the second extremity of the rope
    float thickness;	///<  Thickness of the rope};
  };

  /**
   * \brief  Alga hanging on a line
   */
  struct Alga
  {
    /**
     * \brief  Position of the alga on the line
     *
     * 0.0 -> center of the alga on one extremity of the line <br/>
     * 1.0 -> center of the alga on the other extremity of the line
     */
    tf2::Vector3 position;

    float orientation;  ///<  Orientation of the alga
    float length;		    ///<  Length of the alga
    float width;		    ///<  Width of the alga

    // heatmap color, image texture
    // TODO
  };

  /**
   * \brief  Line on which algae grow
   */
  struct AlgaeLine
  {
    Rope line;					      ///<  Rope on wich the algae grow
    tf2::Vector3 anchor1;		  ///<  First anchor
    tf2::Vector3 anchor2;		  ///<  Second anchor
    float anchors_diameter;	  ///<  Anchors diameter
    float anchors_height;		  ///<  Anchors height
    std::vector<Alga> algae;	///<  List of algae hanging on the line

    // TODO: function mapping (0, 1) to (extremity1, extremity2)
    // TODO: function giving the orientation of the line (to get the orientation of one alga)
  };

  /**
   * \brief  Displays a vector
   *
   * \param stream  Stream on which to display the vector
   * \param v       Vector to display
   * \return  Output stream
   */
  std::ostream &operator<<(std::ostream &stream, const tf2::Vector3 &v);

  /**
   * \brief  Draw a random number from a Gaussian distribution
   *
   * \param mu     Mean of the distribution
   * \param sigma  Standard deviation of the distribution
   * \return  Random number following a normal law (mu, sigma)
   */
  double random_gaussian(double mu, double sigma);

  /**
   * \brief  Draw a random number from a uniform distribution
   *
   * \param a  Lower bound
   * \param b  Upper bound
   * \return   Random number following a uniform law in [a, b]
   */
  double random_uniform(double a, double b);



} // namespace mfcpp

#endif
