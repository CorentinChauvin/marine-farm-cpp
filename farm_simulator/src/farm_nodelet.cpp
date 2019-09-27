/**
 * @file
 *
 * \brief  Definition of nodelet for managing the farm simulation
 * \author Corentin Chauvin-Hamea
 * \date   2019
 */

#include "farm_nodelet.hpp"
#include "farm_common.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>

using namespace std;


PLUGINLIB_EXPORT_CLASS(mfcpp::FarmNodelet, nodelet::Nodelet)

namespace mfcpp {

FarmNodelet::FarmNodelet() {}
FarmNodelet::~FarmNodelet() {}

void FarmNodelet::onInit()
{
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    // ROS publishers
    rviz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("anchors", 0 );

    // ROS parameters
    private_nh_.param<float>("main_loop_freq", main_loop_freq_, 1.0);

    run_nodelet();
}

void FarmNodelet::run_nodelet()
{
    ros::Duration(2.0).sleep(); // wait for Rviz to start
    disp_static_obj();

    ros::Rate loop_rate(main_loop_freq_);
    while (ros::ok()) {
        ros::spinOnce();

        // TODO: do stuff
        // ...

        cout << ros::ok() << endl;
        loop_rate.sleep();
    }
}

void FarmNodelet::disp_static_obj() const
{
    // Initialise common marker data
    MarkerArgs args;
    args.stamp = ros::Time::now();
    args.duration = ros::Duration(0);  // forever
    args.ns = "ns";
    args.frame_id = "/world";

    // Create and publish markers of static objects
    visualization_msgs::MarkerArray markers;

    Rope rope;
    rope.extremity1 = tf2::Vector3(0, 0, 0);
    rope.extremity2 = tf2::Vector3(1, 1, 1);
    rope.thickness = 1.0;
    markers.markers.push_back(rviz_marker(rope, args));

    rviz_pub_.publish(markers);

}

void FarmNodelet::disp_dynamic_obj(float duration) const
{

}

visualization_msgs::Marker FarmNodelet::rviz_marker(const Rope &rope, const MarkerArgs &args) const
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = args.frame_id;
    marker.header.stamp = args.stamp;
    marker.ns = args.ns;
    marker.id = 0;
    marker.lifetime = args.duration;

    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point p;
    p.x = rope.extremity1.getX();
    p.y = rope.extremity1.getY();
    p.z = rope.extremity1.getZ();
    marker.points.push_back(p);
    p.x = rope.extremity2.getX();
    p.y = rope.extremity2.getY();
    p.z = rope.extremity2.getZ();
    marker.points.push_back(p);

    marker.scale.x = rope.thickness;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    return marker;
}

} // namespace mfcpp
