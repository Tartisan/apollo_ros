add_subdirectory(proto)

if(NOT NO_GPU)
  set(PERCEPTION_NODE_LIBRARIES 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    perception_base
    perception_common
    perception_inference
    perception_proto
    perception_lib
    perception_map
  )

  add_library(perception_onboard
    common_flags/common_flags.cc
    msg_serializer/msg_serializer.cc
    transform_wrapper/transform_wrapper.cc
  )
  target_link_libraries(perception_onboard
    perception_onboard_proto
    ${PERCEPTION_NODE_LIBRARIES}
  )

  add_executable(perception_camera_node 
    component/fusion_camera_detection_component.cc
    perception_camera_node.cc 
  )
  target_link_libraries(perception_camera_node 
    perception_camera
    ${PERCEPTION_NODE_LIBRARIES}
  )

  add_library(lidar_detection_component
    component/detection_component.cc
    component/recognition_component.cc
    component/fusion_component.cc
  )
  target_link_libraries(lidar_detection_component
    perception_lidar
    perception_onboard
    conversion
    transform_buffer
    ${PERCEPTION_NODE_LIBRARIES}
  )

  add_executable(perception_lidar_node perception_lidar_node.cc)
  target_link_libraries(perception_lidar_node lidar_detection_component)

  add_executable(lidar_detection_evaluate lidar_detection_evaluate_node.cc)
  target_link_libraries(lidar_detection_evaluate lidar_detection_component absl::strings)
endif()