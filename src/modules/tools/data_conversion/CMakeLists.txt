cmake_minimum_required(VERSION 3.0.2)
project(data_conversion)

set(${PROJECT_NAME}_CATKIN_DEPS
  roscpp
  rosbag
  sensor_msgs
  geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_CATKIN_DEPS}
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

add_library(conversion convert.cc)
target_link_libraries(conversion
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
add_dependencies(conversion localization_generate_messages drivers_generate_messages)

# rosbag_to_record
add_executable(rosbag_to_record rosbag_to_record.cc channel_info.cc)
target_link_libraries(rosbag_to_record conversion)

# record_to_rosbag
add_executable(record_to_rosbag record_to_rosbag.cc)
target_link_libraries(record_to_rosbag conversion)