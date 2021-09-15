cmake_minimum_required(VERSION 3.0.2)
project(data_conversion)

set(${PROJECT_NAME}_CATKIN_DEPS
  roscpp
  rosbag 
  # pcl_ros
  sensor_msgs
  geometry_msgs
  # tf
  # tf_conversions
  # cv_bridge
)

find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_CATKIN_DEPS}
  # pcl_conversions
)

catkin_package(
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
  DEPENDS 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/../devel/include
)

link_directories(${CMAKE_SOURCE_DIR}/../devel/lib/) 

add_executable(rosbag_to_record
  rosbag_to_record/channel_info.cc
  rosbag_to_record/rosbag_to_record.cc
)
target_link_libraries(rosbag_to_record
  ${catkin_LIBRARIES}
  cyber
  cyber_proto
  canbus_proto
  drivers_proto
  localization_proto
  map_proto
  perception_proto
  transform_proto
)