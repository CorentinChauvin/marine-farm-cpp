cmake_minimum_required(VERSION 2.8.3)
project(mf_farm_simulator)

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
  std_msgs
  geometry_msgs
  message_generation
  tf2_geometry_msgs
  visualization_msgs
  dynamic_reconfigure
  mf_common
)

generate_dynamic_reconfigure_options(
  cfg/farm_simulator.cfg
)

add_message_files(
  FILES
    Alga.msg
    Algae.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
    mf_common
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    nodelet roscpp std_msgs geometry_msgs message_runtime mf_common
)

# add_compile_options(-pg)
# set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)


###########
## Build ##
###########

include_directories(
  include/mf_farm_simulator
  ${catkin_INCLUDE_DIRS}
)

add_library(farm_nodelet
  src/mf_farm_simulator/farm_nodelet.cpp
  src/mf_farm_simulator/farm_initialisation.cpp
  src/mf_farm_simulator/farm_output.cpp
  src/mf_farm_simulator/farm_common.cpp
  src/mf_farm_simulator/rviz_visualisation.cpp
  )

add_dependencies(farm_nodelet
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
  mf_common_generate_messages_cpp
)

target_link_libraries(farm_nodelet ${catkin_LIBRARIES})
