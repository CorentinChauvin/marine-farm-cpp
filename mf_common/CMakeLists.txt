cmake_minimum_required(VERSION 2.8.3)
project(mf_common)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(EIGEN_BUILD_FLAGS "-O3 -DEIGEN_NO_DEBUG")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${EIGEN_BUILD_FLAGS}")
set(ROS_BUILD_TYPE Release)

########################
##  Configure Catkin  ##
########################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
    Float32Array.msg
    EulerPose.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime
  LIBRARIES mf_common
)

###########
## Build ##
###########
include_directories(
  include/mf_common
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

add_library(mf_common
  src/mf_common/common.cpp
  src/mf_common/spline.cpp
)

add_dependencies(mf_common
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(mf_common ${catkin_LIBRARIES})
