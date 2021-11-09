cmake_minimum_required(VERSION 2.8.3)
project(common)

find_package(catkin REQUIRED COMPONENTS roscpp)

include_directories(
  ${CMAKE_SOURCE_DIR}/
  ${CMAKE_SOURCE_DIR}/../devel/include/
)

add_subdirectory(proto)

add_library(${PROJECT_NAME}
  # configs
  configs/config_gflags.cc
  # monitor_log
  # monitor_log/monitor_log_buffer.cc
  # monitor_log/monitor_logger.cc
  # util
  util/util.cc
)