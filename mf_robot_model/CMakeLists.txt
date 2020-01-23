cmake_minimum_required(VERSION 2.8.3)
project(mf_robot_model)

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
  roscpp
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  LIBRARIES mf_robot_model
)

# add_compile_options(-pg)
# set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)


###########
## Build ##
###########
include_directories(
  include/mf_robot_model
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

add_library(mf_robot_model
  src/mf_robot_model/robot_model.cpp
)

add_dependencies(mf_robot_model
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mf_robot_model ${catkin_LIBRARIES})
