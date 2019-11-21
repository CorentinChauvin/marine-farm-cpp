cmake_minimum_required(VERSION 2.8.3)
project(mf_sensors_simulator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
# set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer")
# set(ROS_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -Wall")


########################
##  Configure Catkin  ##
########################

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  reactphysics3d
  mf_farm_simulator
  tf2
  tf2_ros
  message_generation
)

add_message_files(
  FILES
    CameraOutput.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS nodelet roscpp message_runtime
)

###########
## Build ##
###########

include_directories(
  include/mf_sensors_simulator
  ${catkin_INCLUDE_DIRS}
)

# Camera nodelet
add_library(camera_nodelet
  src/mf_sensors_simulator/camera_nodelet.cpp
  src/mf_sensors_simulator/collision.cpp
  src/mf_sensors_simulator/output.cpp
)
add_dependencies(camera_nodelet
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(camera_nodelet ${catkin_LIBRARIES})
