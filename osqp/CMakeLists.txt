cmake_minimum_required(VERSION 2.8.3)
project(osqp)

# add_compile_options(-std=c++11)
set(EIGEN_BUILD_FLAGS "-O3 -DEIGEN_NO_DEBUG")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${EIGEN_BUILD_FLAGS}")
set(ROS_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


########################
##  Configure Catkin  ##
########################
find_package(catkin REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS
    include/osqp/include/
    include/osqp_eigen/
    include/
  LIBRARIES
    osqp
    osqp_eigen
  DEPENDS
    EIGEN3
)

################
## Build osqp ##
################

include_directories(
  include/osqp/include
  ${catkin_INCLUDE_DIRS}
)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/include/osqp/configure/cmake)
INCLUDE(FindPythonModule)
INCLUDE(Utils)
option (DFLOAT "Use float numbers instead of doubles" ON)
# option (PRINTING "Enable solver printing" ON)
# option (PROFILING "Enable solver profiling (timing)" ON)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/osqp/configure/osqp_configure.h.in
               ${CMAKE_CURRENT_SOURCE_DIR}/include/osqp/include/osqp_configure.h
               NEWLINE_STYLE LF)

add_subdirectory(include/osqp/lin_sys)
add_subdirectory(include/osqp/src)
add_subdirectory(include/osqp/include)

add_library(osqp SHARED ${osqp_src} ${osqp_headers} ${linsys_solvers})

target_include_directories(osqp PRIVATE ${linsys_solvers_includes})
target_include_directories(osqp PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/osqp/src>"
  "$<INSTALL_INTERFACE:include/osqp>"
)

add_dependencies(osqp ${catkin_EXPORTED_TARGETS})
target_link_libraries(osqp ${catkin_LIBRARIES})


#######################################
## Build osqp-eigen helper functions ##
#######################################

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED 11)

include_directories(
  include/osqp_eigen
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(osqp_eigen
  src/osqp_eigen/osqp_eigen.cpp
)

add_dependencies(osqp_eigen
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(osqp_eigen ${catkin_LIBRARIES} osqp)
