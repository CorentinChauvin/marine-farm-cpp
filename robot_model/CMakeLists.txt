cmake_minimum_required(VERSION 2.8.3)
project(robot_model)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer")
set(ROS_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

########################
##  Configure Catkin  ##
########################
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  LIBRARIES robot_model
)

# add_compile_options(-pg)
# set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)


###########
## Build ##
###########
include_directories(
  include/robot_model
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

add_library(robot_model
  src/robot_model/robot_model.cpp
)

add_dependencies(robot_model
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(robot_model ${catkin_LIBRARIES})
