cmake_minimum_required(VERSION 2.8.3)
project(mf_experiments)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


########################
##  Configure Catkin  ##
########################

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  geometry_msgs
  nav_msgs
  mf_common
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    mf_common
)


###########
## Build ##
###########

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

include_directories(
  include/mf_experiments
  ${catkin_INCLUDE_DIRS}
)

# Control tester
add_executable(control_tester
  src/mf_experiments/control_tester.cpp
)
add_dependencies(control_tester
  ${catkin_EXPORTED_TARGETS}
    mf_common_generate_messages_cpp
)
target_link_libraries(control_tester ${catkin_LIBRARIES})
