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

#imm_motion_predict
add_executable(imm_motion_predict
        src/imm_motion_predict_node.cpp
        src/imm_motion_predict.cpp)

target_link_libraries(imm_motion_predict
        ${catkin_LIBRARIES}
        )

add_dependencies(imm_motion_predict
        ${catkin_EXPORTED_TARGETS}
        )

install(TARGETS
        imm_motion_predict
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
