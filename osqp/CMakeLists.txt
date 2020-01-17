cmake_minimum_required(VERSION 2.8.3)
project(osqp)

# add_compile_options(-std=c++11)
set(EIGEN_BUILD_FLAGS "-O3 -DEIGEN_NO_DEBUG")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${EIGEN_BUILD_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(ROS_BUILD_TYPE Release)


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
    include/osqp-eigen/include
  LIBRARIES
    osqp
    osqp-eigen
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

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/include/osqp/configure/cmake)
include(FindPythonModule)
include(Utils)
option(PROFILING "Enable solver profiling (timing)" ON)

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

######################
## Build osqp-eigen ##
######################

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED 11)

include_directories(
  include/osqp-eigen/include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${EIGEN3_INCLUDE_DIR})

set(osqp-eigen_src
  include/osqp-eigen/src/Data.cpp
  include/osqp-eigen/src/Settings.cpp
  include/osqp-eigen/src/Solver.cpp)

set(osqp-eigen_headers
  include/osqp-eigen/include/OsqpEigen/OsqpEigen.h
  include/osqp-eigen/include/OsqpEigen/Constants.hpp
  include/osqp-eigen/include/OsqpEigen/SparseMatrixHelper.hpp
  include/osqp-eigen/include/OsqpEigen/SparseMatrixHelper.tpp
  include/osqp-eigen/include/OsqpEigen/Data.hpp
  include/osqp-eigen/include/OsqpEigen/Data.tpp
  include/osqp-eigen/include/OsqpEigen/Settings.hpp
  include/osqp-eigen/include/OsqpEigen/Solver.hpp
  include/osqp-eigen/include/OsqpEigen/Solver.tpp)

add_library(osqp-eigen SHARED ${osqp-eigen_src} ${osqp-eigen_headers})

target_include_directories(osqp-eigen PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/osqp-eigen/src>"
  "$<INSTALL_INTERFACE:include/osqp-eigen>"
)

add_dependencies(osqp-eigen ${catkin_EXPORTED_TARGETS})
target_link_libraries(osqp-eigen ${catkin_LIBRARIES} osqp)
