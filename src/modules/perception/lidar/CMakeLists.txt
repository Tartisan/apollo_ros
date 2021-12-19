## proto
add_subdirectory(app/proto)
add_subdirectory(common/proto)
add_subdirectory(lib/feature_extractor/tfe/proto)
add_subdirectory(lib/obstacle/detector/proto)
add_subdirectory(lib/obstacle/postprocessor/location_refiner/proto)
add_subdirectory(lib/obstacle/tracker/omt/proto)
add_subdirectory(lib/obstacle/transformer/multicue/proto)

## obstacle_camera_perception
cuda_add_library(perception_camera 
  # app
  app/debug_info.cc
  app/obstacle_camera_perception.cc
  # common
  common/camera_ground_plane.cc
  common/data_provider.cc
  common/object_template_manager.cc
  common/undistortion_handler.cc
  common/util.cc
  # lib/calibration_service
  lib/calibration_service/online_calibration_service/online_calibration_service.cc
  # lib/calibrator
  lib/calibrator/common/histogram_estimator.cc
  lib/calibrator/laneline/lane_based_calibrator.cc
  lib/calibrator/laneline/laneline_calibrator.cc
  # lib/feature
  lib/feature_extractor/tfe/external_feature_extractor.cc
  lib/feature_extractor/tfe/project_feature.cc
  lib/feature_extractor/tfe/tracking_feat_extractor.cc
  # lib/obstacle
  lib/obstacle/detector/smoke/region_output.cc
  lib/obstacle/detector/smoke/region_output.cu
  lib/obstacle/detector/smoke/smoke_obstacle_detector.cc
  lib/obstacle/detector/yolo/region_output.cc
  lib/obstacle/detector/yolo/region_output.cu
  lib/obstacle/detector/yolo/yolo_obstacle_detector.cc
  lib/obstacle/postprocessor/location_refiner/location_refiner_obstacle_postprocessor.cc
  lib/obstacle/postprocessor/location_refiner/obj_postprocessor.cc
  lib/obstacle/tracker/common/kalman_filter.cc
  lib/obstacle/tracker/common/similar.cu
  lib/obstacle/tracker/omt/obstacle_reference.cc
  lib/obstacle/tracker/omt/omt_obstacle_tracker.cc
  lib/obstacle/tracker/omt/target.cc
  lib/obstacle/tracker/omt/track_object.cc
  lib/obstacle/transformer/multicue/multicue_obstacle_transformer.cc
  lib/obstacle/transformer/multicue/obj_mapper.cc
  # tools
  tools/transform_server.cc
  tools/visualizer.cc
)
target_link_libraries(perception_camera 
  ${CUDA_LIBRARIES}
  perception_proto
  perception_base
  perception_common
  camera_app_proto
  camera_common_proto
  camera_lib_feature_extractor_tfe_proto
  camera_lib_detector_proto
  camera_lib_postprocessor_location_refiner_proto
  camera_lib_tracker_omt_proto
  camera_lib_transformer_multicue_proto
  nvinfer_plugin 
  nvinfer
)
