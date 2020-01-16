cmake_minimum_required(VERSION 2.8.3)
project(osqp)

# add_compile_options(-std=c++11)
set(ROS_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


########################
##  Configure Catkin  ##
########################
find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include/osqp/include/
  LIBRARIES osqp
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
