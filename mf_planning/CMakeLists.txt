cmake_minimum_required(VERSION 2.8.3)
project(mf_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer")
# set(ROS_BUILD_TYPE Debug)

set(EIGEN_BUILD_FLAGS "-O3 -DEIGEN_NO_DEBUG")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${EIGEN_BUILD_FLAGS}")
set(ROS_BUILD_TYPE Release)


########################
##  Configure Catkin  ##
########################

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS nodelet roscpp
)


###########
## Build ##
###########

include_directories(
  include/mf_planning
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

# Planning nodelet
add_library(planning_nodelet
  src/mf_planning/planning_nodelet.cpp
)
add_dependencies(planning_nodelet
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(planning_nodelet ${catkin_LIBRARIES})
