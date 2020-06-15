cmake_minimum_required(VERSION 2.8.3)
project(mf_experiments)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


########################
##  Configure Catkin  ##
########################

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  roslib
  geometry_msgs
  nav_msgs
  mf_common
  mf_farm_simulator
  mf_mapping
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    mf_common
    mf_farm_simulator
    mf_mapping
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

# Trajectory publisher
add_executable(traj_publisher
  src/mf_experiments/traj_publisher.cpp
)
add_dependencies(traj_publisher
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(traj_publisher ${catkin_LIBRARIES})

# Experiment statistics
add_executable(experiment_stats
  src/mf_experiments/experiment_stats.cpp
)
add_dependencies(experiment_stats
  ${catkin_EXPORTED_TARGETS}
    mf_common_generate_messages_cpp
    mf_mapping_generate_messages_cpp
)
target_link_libraries(experiment_stats ${catkin_LIBRARIES})

# Clock publisher
add_executable(clock_publisher
  src/mf_experiments/clock_publisher.cpp
)
add_dependencies(clock_publisher
  ${catkin_EXPORTED_TARGETS}
    mf_common_generate_messages_cpp
)
target_link_libraries(clock_publisher ${catkin_LIBRARIES})
