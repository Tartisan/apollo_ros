cmake_minimum_required(VERSION 2.8.3)
project(drivers)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  ContiRadar.msg
  ContiRadarObs.msg
  ClusterListStatus_600.msg
  ObjectListStatus_60A.msg
  RadarState_201.msg
)
generate_messages(DEPENDENCIES std_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/
  ${CMAKE_SOURCE_DIR}/../devel/include/
  ${EIGEN3_INCLUDE_DIR}
)

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  set(HERMES_LIB_DIR ${CMAKE_SOURCE_DIR}/third_party/can_card_library/hermes_can/lib_x86_64)
else()
  set(HERMES_LIB_DIR ${CMAKE_SOURCE_DIR}/third_party/can_card_library/hermes_can/lib_aarch64)
endif()

link_directories(
  ${CMAKE_SOURCE_DIR}/../devel/lib/
  ${CMAKE_SOURCE_DIR}/third_party/can_card_library/esd_can/lib
  ${HERMES_LIB_DIR}
)

add_subdirectory(proto)
add_subdirectory(canbus)
add_subdirectory(radar/conti_radar)
add_subdirectory(radar/racobit_radar)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs genmsg
  DEPENDS
)
