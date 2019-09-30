#include "ros/ros.h"
#include "nodelet/loader.h"

#include <iostream>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "farm_simulator");
    nodelet::Loader manager;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;

    std::string nodelet_name = ros::this_node::getName();

    manager.load(nodelet_name, "farm_simulator/FarmNodelet", remap, nargv);

    ros::spin();

    return 0;
}
