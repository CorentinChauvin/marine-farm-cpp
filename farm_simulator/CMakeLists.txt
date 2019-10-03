cmake_minimum_required(VERSION 2.8.3)
project(farm_simulator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

########################
##  Configure Catkin  ##
########################
find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  std_msgs
  geometry_msgs
  tf2_geometry_msgs
  visualization_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/farm_simulator.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS nodelet roscpp std_msgs
)



###########
## Build ##
###########
include_directories(
  include/farm_simulator
  include/perlin_noise
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

add_library(farm_nodelet
  src/farm_nodelet.cpp
  src/farm_common.cpp
  src/rviz_visualisation.cpp
  )

add_dependencies(farm_nodelet
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(farm_nodelet ${catkin_LIBRARIES})
