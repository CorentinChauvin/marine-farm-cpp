cmake_minimum_required(VERSION 2.8.3)
project(farm_simulator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer")
set(ROS_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

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

# add_compile_options(-pg)
# set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)


###########
## Build ##
###########

# Build reactphysics3d
set(CMAKE_INSTALL_LIBDIR ${CATKIN_PACKAGE_LIB_DESTINATION}/reactphysics3d)
add_subdirectory(reactphysics3d)

# Build farm simulator
include_directories(
  include/farm_simulator
  include/perlin_noise
  ${catkin_INCLUDE_DIRS}
)

add_library(farm_nodelet
  src/farm_simulator/farm_nodelet.cpp
  src/farm_simulator/farm_visualisation.cpp
  src/farm_simulator/farm_common.cpp
  src/farm_simulator/rviz_visualisation.cpp
  src/perlin_noise/perlin_noise.cpp
  )

add_dependencies(farm_nodelet
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(farm_nodelet ${catkin_LIBRARIES} reactphysics3d)
