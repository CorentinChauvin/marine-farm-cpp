cmake_minimum_required(VERSION 2.8.3)
project(mf_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
# set(CMAKE_CXX_FLAGS "-O2 -g -DNDEBUG -fno-omit-frame-pointer -DEIGEN_NO_DEBUG")
# set(ROS_BUILD_TYPE RelWithDebInfo)

set(EIGEN_BUILD_FLAGS "-O3 -DEIGEN_NO_DEBUG")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${EIGEN_BUILD_FLAGS}")
set(ROS_BUILD_TYPE Release)


########################
##  Configure Catkin  ##
########################

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  mf_sensors_simulator
  tf2
  tf2_ros
  geometry_msgs
  sensor_msgs
  message_generation
  mf_common
)

add_service_files(
  FILES
    UpdateGP.srv
    LoadGP.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    mf_sensors_simulator
    mf_common
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS nodelet roscpp message_runtime
)


###########
## Build ##
###########

include_directories(
  include/mf_mapping
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

# Gaussian Process nodelet
add_library(gp_nodelet
  src/mf_mapping/gp_nodelet.cpp
  src/mf_mapping/gp.cpp
  src/mf_mapping/gp_output.cpp
)
add_dependencies(gp_nodelet
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  mf_sensors_simulator_generate_messages_cpp
  mf_common_generate_messages_cpp
)
target_link_libraries(gp_nodelet ${catkin_LIBRARIES})
