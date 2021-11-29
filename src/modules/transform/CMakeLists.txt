cmake_minimum_required(VERSION 2.8.3)
project(transform)

set(${PROJECT_NAME}_CATKIN_DEPS roscpp tf2 tf2_ros)
find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_CATKIN_DEPS}
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES perception
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
  # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/
  ${CMAKE_SOURCE_DIR}/../devel/include/
  ${YAML_CPP_INCLUDE_DIRS}
)

add_subdirectory(proto)

add_executable(static_transform
  static_transform_node.cc
  static_transform_component.cc
)
target_link_libraries(static_transform 
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  transform_proto
  cyber
)