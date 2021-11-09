cmake_minimum_required(VERSION 2.8.3)
project(localization)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs genmsg)

include_directories(
  ${CMAKE_SOURCE_DIR}/
  ${CMAKE_SOURCE_DIR}/../devel/include/
)

add_subdirectory(proto)

add_message_files(
  FILES
  Pose.msg
  Uncertainty.msg
  LocalizationEstimate.msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs genmsg
  DEPENDS
)