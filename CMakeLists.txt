cmake_minimum_required(VERSION 2.8.3)
project(ik_solver_lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  trac_ik_lib
  kdl_parser
  pluginlib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ik_solver_lib
  CATKIN_DEPENDS roscpp trac_ik_lib kdl_parser pluginlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ik_solver_lib src/trac_ik/trac_ik_solver.cpp)
target_link_libraries(ik_solver_lib ${catkin_LIBRARIES})

