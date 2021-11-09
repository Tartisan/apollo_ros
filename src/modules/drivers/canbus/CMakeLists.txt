add_subdirectory(proto)

include_directories(
  ${CMAKE_SOURCE_DIR}/third_party/can_card_library
)

add_library(drivers_canbus
  sensor_gflags.cc
  # can_client
  # can_client/esd/esd_can_client.cc
  can_client/fake/fake_can_client.cc
  can_client/hermes_can/hermes_can_client.cc
  can_client/socket/socket_can_client_raw.cc
  can_client/can_client_factory.cc
  can_client/can_client_tool.cc
  # can_comm
  # common
  common/byte.cc
)
target_link_libraries(drivers_canbus 
  common_proto
  # ntcan
  bcan
)