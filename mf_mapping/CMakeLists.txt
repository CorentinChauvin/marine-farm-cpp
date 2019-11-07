cmake_minimum_required(VERSION 2.8.3)
project(mf_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


########################
##  Configure Catkin  ##
########################

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  sensors_simulator
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS nodelet roscpp
)


###########
## Build ##
###########

include_directories(
  include/mf_mapping
  ${catkin_INCLUDE_DIRS}
)

# Gaussian Process nodelet
add_library(gp_nodelet
  src/mf_mapping/gp_nodelet.cpp
  src/mf_mapping/gp.cpp
)
add_dependencies(gp_nodelet
  ${catkin_EXPORTED_TARGETS}
  sensors_simulator_generate_messages_cpp
)
target_link_libraries(gp_nodelet ${catkin_LIBRARIES})
