
add_library(perception_common
  sensor_manager/sensor_manager.cc
  io/io_util.cc
  perception_gflags.cc
)
target_link_libraries(perception_common 
  ${YAML_CPP_LIBRARIES}
  perception_base
  perception_proto
  boost_filesystem
)
