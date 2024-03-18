cmake_minimum_required(VERSION 3.0.2)
project(rm_local_planner)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(casadi REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roborts_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  dynamic_reconfigure
  tf
  tf2_ros
  base_local_planner
  local_planner
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rm_local_planner
CATKIN_DEPENDS local_planner
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# link_directories(/usr/local/lib)
# include_directories(/usr/local/include/yaml-cpp)
# include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rm_local_planner.cpp
# )

add_library(${PROJECT_NAME}
  src/mpc_controller_osqp/mpc_controller_osqp.cpp
  src/mpc_controller_casadi/mpc_controller_casadi.cpp
  src/LQR_controller/LQR_controller.cpp
  # src/mpc_controller_cppad/mpc_controller_cppad.cpp
)



target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  OsqpEigen
  ipopt 
  casadi 
)


add_executable(rm_local_planner_node src/control_base.cpp
 )

target_link_libraries(rm_local_planner_node
  ${catkin_LIBRARIES}
  OsqpEigen
  ${PROJECT_NAME}
  ipopt 
  casadi 
)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


