cmake_minimum_required(VERSION 2.8.3)
project(common)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Eigen3 REQUIRED)

include_directories(
  ${CMAKE_SOURCE_DIR}/
  ${CMAKE_SOURCE_DIR}/../devel/include/
  ${EIGEN3_INCLUDE_DIR}
)

add_subdirectory(proto)
add_subdirectory(configs/proto)

add_library(${PROJECT_NAME}
  # adapters
  adapters/adapter_gflags.cc
  # configs
  configs/config_gflags.cc
  configs/vehicle_config_helper.cc
  # math
  math/aabox2d.cc
  math/box2d.cc
  math/line_segment2d.cc
  math/linear_interpolation.cc
  math/math_utils.cc
  math/polygon2d.cc
  math/vec2d.cc
  # monitor_log
  # monitor_log/monitor_log_buffer.cc
  # monitor_log/monitor_logger.cc
  # util
  util/util.cc
)
target_link_libraries(${PROJECT_NAME} common_proto common_configs_proto)