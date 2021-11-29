add_subdirectory(proto)

add_executable(conti_radar
  conti_radar_canbus_node.cc
  conti_radar_canbus_component.cc
  conti_radar_message_manager.cc
  # protocol
  protocol/cluster_general_info_701.cc
  protocol/cluster_list_status_600.cc
  protocol/cluster_quality_info_702.cc
  protocol/motion_input_speed_300.cc
  protocol/motion_input_yawrate_301.cc
  protocol/object_extended_info_60d.cc
  protocol/object_general_info_60b.cc
  protocol/object_list_status_60a.cc
  protocol/object_quality_info_60c.cc
  protocol/radar_config_200.cc
  protocol/radar_state_201.cc
)
target_link_libraries(conti_radar 
  ${catkin_LIBRARIES}
  cyber
  conversion
  drivers_canbus
)