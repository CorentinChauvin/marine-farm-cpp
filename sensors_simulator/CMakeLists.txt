cmake_minimum_required(VERSION 2.8.3)
project(sensors_simulator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
# set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer")
# set(ROS_BUILD_TYPE Debug)


########################
##  Configure Catkin  ##
########################

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  reactphysics3d
  farm_simulator
  tf2
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS nodelet roscpp
)

###########
## Build ##
###########

include_directories(
  include/sensors_simulator
  ${catkin_INCLUDE_DIRS}
)

# Camera nodelet
add_library(camera_nodelet
  src/sensors_simulator/camera_nodelet.cpp
  src/sensors_simulator/collision.cpp
  src/sensors_simulator/output.cpp
)

add_dependencies(camera_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_nodelet ${catkin_LIBRARIES})
