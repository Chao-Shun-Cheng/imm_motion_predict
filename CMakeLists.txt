cmake_minimum_required(VERSION 2.8.3)
project(imm_motion_predict)

include(FindPkgConfig)

find_package(autoware_build_flags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  imm_ukf_pda_track
  autoware_msgs
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  autoware_msgs
  tf
  imm_ukf_pda_track
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

