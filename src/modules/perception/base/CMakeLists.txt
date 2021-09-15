## perception_base
add_library(perception_base
  blob.cc
  camera.cc
  distortion_model.cc
  object.cc
  omnidirectional_model.cc
  point_cloud_util.cc
  polynomial
  syncedmem.cc
)
target_link_libraries(perception_base
  ${CUDA_LIBRARIES}
  ${OpenCV_LIBRARIES}
)