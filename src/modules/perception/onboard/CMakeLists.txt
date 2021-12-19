add_executable(perception_camera_node 
  component/fusion_camera_detection_component.cc
  perception_camera_node.cc 
)
target_link_libraries(perception_camera_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  perception_base
  perception_camera
  perception_common
  perception_inference
  perception_proto
  perception_lib
)

# add_executable(perception_lidar 
#   onboard/component/detection_component.cc
#   onboard/component/recognition_component.cc
#   onboard/component/fusion_component.cc
#   onboard/perception_lidar_node.cc 
# )
# target_link_libraries(perception_lidar
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${CUDA_LIBRARIES}
#   ${PROTOBUF_LIBRARIES}
#   ${YAML_CPP_LIBRARIES}
#   perception_base
#   perception_camera
#   perception_common
#   perception_inference
#   perception_proto
#   perception_lib
# )