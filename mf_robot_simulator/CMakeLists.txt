cmake_minimum_required(VERSION 2.8.3)
project(mf_robot_simulator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
# set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer")
# set(ROS_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

set(ROS_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -Wall")


########################
##  Configure Catkin  ##
########################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mf_robot_model
  std_msgs
  message_generation
  tf
)

add_message_files(
  FILES
    Command.msg
    CartesianCommand.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime
)


#############
##  Build  ##
#############
include_directories(
  include/mf_robot_simulator
  ${mf_robot_model_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_simulator
  src/mf_robot_simulator/robot_simulator.cpp
)
add_dependencies(robot_simulator
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(robot_simulator ${catkin_LIBRARIES})
