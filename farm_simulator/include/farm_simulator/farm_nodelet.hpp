/**
 * @file
 *
 * \brief  Declaration of nodelet for managing the farm simulation
 * \author Corentin Chauvin-Hamea
 * \date   2019
 */

#ifndef FARM_NODELET_HPP
#define FARM_NODELET_HPP

#include "farm_common.hpp"
#include <visualization_msgs/Marker.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <string>
#include <vector>

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
            /**
             * \brief  Arguments used to create a Rviz maker
             */
            struct MarkerArgs {
                /// Time stamp for the ROS message
                ros::Time stamp;

                /// Frame in which position/orientation of the object is
                std::string frame_id;

                /// Namespace for the Rviz marker
                std::string ns;

                /// Duration of the marker (in sec)
                ros::Duration duration;
            };

            /// Node handler (for topics and services)
            ros::NodeHandle nh_;

            /// Private node handler (for parameters)
            ros::NodeHandle private_nh_;

            /// ROS publisher for Rviz
            ros::Publisher rviz_pub_;

            /// Frequency of the main loop
            float main_loop_freq_;

            /// Vector of all the algae in the farm
            std::vector<Alga> algae;

            /**
             * \brief  Main loop of the nodelet
             */
            void run_nodelet();

            /**
             * \brief  Displays static objects by publishing permanent Rviz markers
             */
            void disp_static_obj() const;

            /**
             * \brief  Displays dynamic objects by publishing temporary Rviz markers
             *
             * \param duration  Duration of the marker (in sec)
             */
            void disp_dynamic_obj(float duration) const;

            /**
             * \brief  Creates a Rviz marker to display a rope
             *
             * \param rope          Rope to display
             * \return out_marker   Corresponding Rviz marker
             */
            visualization_msgs::Marker rviz_marker(const Rope &rope, const MarkerArgs &args) const;

    };



} // namespace mfcpp

#endif
