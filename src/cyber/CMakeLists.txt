cmake_minimum_required(VERSION 2.8.3)
project(cyber)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES perception
  CATKIN_DEPENDS roscpp
  # DEPENDS system_lib
)

include_directories(
  ${CMAKE_SOURCE_DIR}/
  ${CMAKE_SOURCE_DIR}/../devel/include/
)

add_subdirectory(proto)

add_library(${PROJECT_NAME} 
  # common
  common/file.cc
  # message
  message/protobuf_factory.cc
  # record
  record/file/record_file_base.cc
  record/file/record_file_reader.cc
  record/file/record_file_writer.cc
  record/header_builder.cc
  record/record_reader.cc
  record/record_viewer.cc
  record/record_writer.cc
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_proto
  glog
)
