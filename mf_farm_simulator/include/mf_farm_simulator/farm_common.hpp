/**
 * @file
 *
 * \brief  Declaration of common structures and functions for farm simulator
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#ifndef FARM_COMMON_HPP
#define FARM_COMMON_HPP

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Point32.h>
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

    /**
     * \brief  Heatmap representing disease across the alga
     *
     * 0.0 -> alga sane <br/>
     * 1.0 -> alga sick
     */
    std::vector<std::vector<float>> disease_heatmap;
  };

  /**
   * \brief  Line on which algae grow
   */
  struct AlgaeLine
  {
    Rope line;					      ///<  Rope on wich the algae grow
    Rope floating_rope;       ///<  Rope on wich the buoys are
    tf2::Vector3 anchor1;		  ///<  First anchor
    tf2::Vector3 anchor2;		  ///<  Second anchor
    float thickness_ropes;    ///<  Thickness of each rope
    float anchors_diameter;	  ///<  Anchors diameter
    float anchors_height;		  ///<  Anchors height
    unsigned int nbr_buoys;   ///<  Number of buoys on the floating rope
    float buoys_diameter;     ///<  Diameter of a buoy
    std::vector<Alga> algae;	///<  List of algae hanging on the line
  };

  /**
   * \brief  Computes the corner coordinates of an alga
   *
   * coord[0] | coord[1]   --> line
   * -------------------
   * coord[3] | coord[2]
   *
   * \param line  Algae line on which the algae is
   * \param alga  Alga to find the coordinates
   * \return  Coordinates of the alga
   */
  std::vector<tf2::Vector3> get_alga_coord(const AlgaeLine &line, const Alga &alga);


  /**
   * \brief  Displays a vector
   *
   * \param stream  Stream on which to display the vector
   * \param v       Vector to display
   * \return  Output stream
   */
  std::ostream &operator<<(std::ostream &stream, const tf2::Vector3 &v);

  /**
   * \brief  Convert a tf2::Vector3 to a geometry_msgs::Point32
   *
   * \note   tf2::ToMsg() implements a conversion to Point, but not Point32
   * \param p  Vector3 to convert
   * \return  Converted Point32
   */
  inline geometry_msgs::Point32 vector3_to_point32(tf2::Vector3 p)
  {
    geometry_msgs::Point32 pt;
    pt.x = p.getX();
    pt.y = p.getY();
    pt.z = p.getZ();

    return pt;
  }

  /**
   * \brief  Draw a random number from a Gaussian distribution
   *
   * \param mu     Mean of the distribution
   * \param sigma  Standard deviation of the distribution
   * \return  Random number following a normal law (mu, sigma)
   */
  double rand_gaussian(double mu, double sigma);

  /**
   * \brief  Draw a random number from a uniform distribution
   *
   * \param a  Lower bound
   * \param b  Upper bound
   * \return   Random number following a uniform law in [a, b]
   */
  double rand_uniform(double a, double b);

  /**
   * \brief  Draw a random number from a bernoulli distribution
   *
   * \param p  Probability to get true
   * \return   Random number following a uniform law in [a, b]
   */
  bool rand_bernoulli(double p);



} // namespace mfcpp

#endif
