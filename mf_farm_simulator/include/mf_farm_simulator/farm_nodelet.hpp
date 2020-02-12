/**
 * @file
 *
 * \brief  Declaration of nodelet for managing the farm simulation
 * \author Corentin Chauvin-Hameau
 * \date   2019
 */

#ifndef FARM_NODELET_HPP
#define FARM_NODELET_HPP

#include "farm_common.hpp"
#include "rviz_visualisation.hpp"
#include "mf_farm_simulator/FarmSimulatorConfig.h"
#include "mf_common/perlin_noise.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <cstdlib>
#include <csignal>


namespace mfcpp {
  /**
   * \brief  Nodelet class for farm simulation
   */
  class FarmNodelet: public nodelet::Nodelet {
    public:
      FarmNodelet();
      ~FarmNodelet();

      /**
       * \brief  Function called at beginning of nodelet execution
       */
      virtual void onInit();

    private:
      // Static members
      // Note: the timers need to be static since stopped by the SIGINT callback
      static sig_atomic_t volatile b_sigint_;  ///<  Whether SIGINT signal has been received
      static ros::Timer init_timer_;  ///<  Timer callback for the init function
      static ros::Timer main_timer_;  ///<  Timer callback for the main function

      // Private members
      ros::NodeHandle nh_;            ///<  Node handler (for topics and services)
      ros::NodeHandle private_nh_;    ///<  Private node handler (for parameters)
      dynamic_reconfigure::Server<mf_farm_simulator::FarmSimulatorConfig> reconf_srv_;  ///<  Dynamic reconfigure server
      ros::Publisher rviz_pub_;       ///<  ROS publisher for Rviz
      ros::Publisher algae_pub_;      ///<  ROS publisher for the algae
      std::vector<AlgaeLine> algae_lines_;  ///<  Vector of all the algae in the farm
      bool reconfigure_initialised_;  ///<  Whether the dynamic reconfigure callback has been called once
      bool init_done_;  ///<  Whether the farm initialisation has been done
      std::random_device random_device_;  ///<  Seed initialiser for random number generation
      std::mt19937 random_generator_;     ///<  Random number initialiser
      PerlinNoiseGenerator perlin_;       ///<  For randomising the heatmaps


      /// \name  ROS parameters
      ///@{
      float main_loop_freq_;   ///<  Frequency of the main loop
      int random_seed_;        ///<  Seed for random numbers (0 for random seed)
      int nbr_lines_;          ///<  Number of algae lines
      float offset_lines_;     ///<  Lateral distance (m) between each line
      float length_lines_;     ///<  Length (m) of each line
      float thickness_ropes_;  ///<  Diameter (m) of each line
      float depth_lines_;   ///<  Distance (m) between water surface and line
      float depth_water_;   ///<  Distance (m) between water surface and seafloor
      float anchors_diameter_;  ///<  Diameter (m) of the cylindrical anchors
      float anchors_height_;    ///<  Height (m) of the cylindrical anchors
      int nbr_buoys_;           ///<  Number of buoys on each floating rope
      float buoys_diameter_;    ///<  Diameter (m) of each buoy

      bool randomise_lines_;    ///<  Whether to randomise the position of each line
      float alga_miss_rate_;    ///<  Probability to have a missing alga
      float phi_lines_;         ///<  Mean of phi angle for algae line generation
      float theta_lines_;       ///<  Mean of theta angle for algae line generation
      float bnd_phi_lines_;     ///<  Bound such that phi is sampled in [mean-bnd, mean+bnd]
      float bnd_theta_lines_;   ///<  Bound such that theta is sampled in [mean-bnd, mean+bnd]
      float bnd_gamma_lines_;   ///<  Bound such that gamma is sampled in [mean-bnd, mean+bnd]

      bool randomise_algae_;  ///<  Whether to randomise size and orientation of algae
      int nbr_algae_;         ///<  Number of algae per line
      float width_algae_;     ///<  Mean of the width of an alga
      float length_algae_;    ///<  Mean of the length of an alga
      float thickness_algae_;   ///<  Thickness of an alga (for collision detection)
      float psi_algae_;         ///<  Mean of the algae orientation
      float std_width_algae_;   ///<  Standard deviation on algae width
      float std_length_algae_;  ///<  Standard deviation on algae length
      float std_psi_algae_;     ///<  Standard deviation on algae orientation

      bool disp_disease_;    ///<  Whether to display the disease heatmaps
      float disease_ratio_;  ///<  Ratio of alga disease (0->fully sane, 1->fully sick)
      int height_disease_heatmap_;  ///<  Height of the algae disease heatmap
      int width_disease_heatmap_;   ///<  Width of the algae disease heatmap
      int height_grid_heatmap_;     ///<  Height of the grid for perlin noise generation
      int width_grid_heatmap_;      ///<  Width of the grid for perlin noise generation
      ///@}


      /**
       * \brief  Main callback which is called by a timer
       *
       * \param timer_event  Timer event information
       */
      void main_cb(const ros::TimerEvent& timer_event);

      /**
       * \brief  SINGINT (Ctrl+C) callback to stop the nodelet properly
       */
      static void sigint_handler(int s);

      /**
       * \brief  Callback for dynamic reconfigure
       *
       * \param  New configuration
       * \param  Change level
       */
      void reconfigure_cb(mf_farm_simulator::FarmSimulatorConfig &config,
        uint32_t level);

      /**
       * \brief  Publishes the position and heatmap of the algae
       */
      void pub_algae();


      /// \name  Initialisation functions
      ///@{
      /**
       * \brief  Callback for the oneshot initialisation timer
       *
       * \param timer_event  Timer event information
       */
      void init_cb(const ros::TimerEvent &timer_event);

      /**
       * \brief  Initialise the algae lines
       */
      void init_algae_lines();

      /**
       * \brief  Draw a random number from a Gaussian distribution
       *
       * \param mu     Mean of the distribution
       * \param sigma  Standard deviation of the distribution
       * \return  Random number following a normal law (mu, sigma)
       */
      template <class T>
      inline T rand_gaussian(T mu, T sigma);

      /**
       * \brief  Draw a random number from a uniform distribution
       *
       * \param a  Lower bound
       * \param b  Upper bound
       * \return   Random number following a uniform law in [a, b]
       */
      template <class T>
      inline T rand_uniform(T a, T b);

      /**
       * \brief  Draw a random number from a bernoulli distribution
       *
       * \param p  Probability to get true
       * \return   Random number following a uniform law in [a, b]
       */
      bool rand_bernoulli(double p);

      /**
       * \brief  Init anchors of a specific algae line
       *
       * \param line  Concerned algae line
       * \param i     Index of the algae line
       */
      void init_anchors(AlgaeLine &line, unsigned int i);

      /**
       * \brief  Init ropes of an algae line
       *
       * \param line  Concerned algae line
       */
      void init_ropes(AlgaeLine &line);

      /**
       * \brief  Init algae of an algae line
       *
       * \param line  Concerned algae line
       */
      void init_algae(AlgaeLine &line);


      ///@}

      /// \name  Visualisation functions
      ///@{
      /**
       * \brief  Displays objects by publishing Rviz markers
       *
       * \param duration  Duration of the marker (in sec)
       */
      void pub_rviz_markers(float duration) const;

      /**
       * \brief  Populates a marker for displaying the buoys
       *
       * \param marker  Marker to populate
       * \param args    Common arguments to fill ROS message
       */
      void pop_buoys_marker(visualization_msgs::Marker &marker,
        MarkerArgs args) const;

      /**
       * \brief  Populates a marker for displaying the ropes
       *
       * \param marker  Marker to populate
       * \param args    Common arguments to fill ROS message
       */
      void pop_ropes_marker(visualization_msgs::Marker &marker,
        MarkerArgs args) const;

      /**
       * \brief  Populates a marker for displaying the algae
       *
       * \param marker  Marker to populate
       * \param args    Common arguments to fill ROS message
       */
      void pop_algae_marker(visualization_msgs::Marker &marker,
        MarkerArgs args) const;

      /**
       * \brief  Populates a marker for displaying the disease heatmaps
       *
       * \param marker  Marker to populate
       * \param args    Common arguments to fill ROS message
       */
      void pop_algae_heatmaps(visualization_msgs::Marker &marker,
        MarkerArgs args) const;

      /**
       * \brief  Populates a triangle marker for displaying images
       *
       * coord[0] | coord[1]
       * -------------------
       * coord[3] | coord[2]
       *
       * \note  For speed issues, space for the concerned vectors should
       *        already be reserved.
       *
       * \param marker   Marker to populate
       * \param img      Black and white image to add to the marker
       * \param coord    Four 3D coordinates of the corners of the image
       */
      void pop_img_marker(visualization_msgs::Marker &marker,
        std::vector<std::vector<float>> img,
        const std::vector<tf2::Vector3> &coord) const;
      ///@}

  };

  /*
   * Inline functions
   */

  template <class T>
  inline T FarmNodelet::rand_gaussian(T mu, T sigma)
  {
    std::normal_distribution<T> distribution(mu, sigma);
    return distribution(random_generator_);
  }


  template <class T>
  inline T FarmNodelet::rand_uniform(T a, T b)
  {
    std::uniform_real_distribution<T> distribution(a, b);
    return distribution(random_generator_);
  }


  inline bool FarmNodelet::rand_bernoulli(double p)
  {
    std::bernoulli_distribution distribution(p);
    return distribution(random_generator_);
  }



}  // namespace mfcpp

#endif
