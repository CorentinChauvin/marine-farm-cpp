/**
 * @file
 *
 * \brief  Declaration of useful classes for planning with lattices
 * \author Corentin Chauvin-Hameau
 * \date   2020
 *
 * In our case, a lattice is a set of viewpoints corresponding to one step of
 * the planning. Several lattices can be generated to plan several steps ahead.
 */

#ifndef LATTICE_HPP
#define LATTICE_HPP

#include <geometry_msgs/Pose.h>
#include <vector>


namespace mfcpp {

/**
 * \brief  Class to represent a node (viewpoint) of a lattice
 */
class LatticeNode
{
  public:
    geometry_msgs::Pose pose;   ///<  Pose of the node in robot frame
    float speed;                ///<  Longitudinal speed of the robot at this node
    std::vector<std::shared_ptr<LatticeNode>> next;  ///<  Nodes in the next lattice that can be reached from this node

    float info_gain;            ///<  Information gain fo this viewpoint
    std::vector<float> gp_cov;  ///<  Diagonal of the covariance of the Gaussian Process
    std::vector<float> camera_pts_x;  ///<  X coord of camera hit pts for the viewpoint (in ocean frame)
    std::vector<float> camera_pts_y;  ///<  Y coord of camera hit pts for the viewpoint (in ocean frame)
    std::vector<float> camera_pts_z;  ///<  Z coord of camera hit pts for the viewpoint (in ocean frame)

    LatticeNode() {
      next.resize(0);
      info_gain = 0;
      gp_cov.resize(0);
      camera_pts_x.resize(0);
      camera_pts_y.resize(0);
      camera_pts_z.resize(0);
    }

    ~LatticeNode() {

    }
};

/**
 * \brief  Class to represent a lattice
 */
class Lattice
{
  public:
    std::vector<std::shared_ptr<LatticeNode>> nodes;  ///<  Nodes of the lattice

    Lattice() {
      nodes.resize(0);
    }

    ~Lattice() {

    }
};


}  // namespace mfcpp

#endif
