## proto
add_subdirectory(app/proto)
add_subdirectory(lib/detector/cnn_segmentation/proto)
add_subdirectory(lib/detector/ncut_segmentation/proto)
add_subdirectory(lib/ground_detector/ground_service_detector/proto)
add_subdirectory(lib/ground_detector/spatio_temporal_ground_detector/proto)
add_subdirectory(lib/object_filter_bank/proto)
add_subdirectory(lib/pointcloud_preprocessor/proto)
add_subdirectory(lib/roi_filter/hdmap_roi_filter/proto)
add_subdirectory(lib/scene_manager/proto)
add_subdirectory(lib/scene_manager/ground_service/proto)
add_subdirectory(lib/scene_manager/roi_service/proto)
add_subdirectory(lib/tracker/multi_lidar_fusion/proto)

cuda_add_library(perception_lidar 
  # app
  app/lidar_obstacle_detection.cc
  app/lidar_obstacle_tracking.cc
  # common
  common/cloud_mask.cc
  common/lidar_frame_pool.cc
  common/lidar_object_util.cc
  common/object_sequence.cc
  # lib
  lib/classifier/fused_classifier/ccrf_type_fusion.cc
  lib/classifier/fused_classifier/fused_classifier.cc
  lib/classifier/fused_classifier/util.cc
  lib/detector/cnn_segmentation/cnn_segmentation.cc
  lib/detector/cnn_segmentation/feature_generator.cc
  lib/detector/cnn_segmentation/feature_generator.cu
  lib/detector/cnn_segmentation/spp_engine/spp_cluster_list.cc
  lib/detector/cnn_segmentation/spp_engine/spp_engine.cc
  lib/detector/cnn_segmentation/spp_engine/spp_label_image.cc
  lib/detector/cnn_segmentation/spp_engine/spp_pool_types.cc
  lib/detector/cnn_segmentation/spp_engine/spp_seg_cc_2d.cc
  lib/detector/cnn_segmentation/spp_engine/spp_struct.cc
  lib/detector/mask_pillars_detection/mask_pillars_detection.cc
  lib/detector/ncut_segmentation/ncut_segmentation.cc
  lib/detector/ncut_segmentation/ncut.cc
  lib/detector/ncut_segmentation/common/flood_fill.cc
  lib/detector/point_pillars_detection/anchor_mask_cuda.cu
  lib/detector/point_pillars_detection/nms_cuda.cu
  lib/detector/point_pillars_detection/pfe_cuda.cu
  lib/detector/point_pillars_detection/point_pillars_detection.cc
  lib/detector/point_pillars_detection/point_pillars.cc
  lib/detector/point_pillars_detection/postprocess_cuda.cu
  lib/detector/point_pillars_detection/preprocess_points_cuda.cu
  lib/detector/point_pillars_detection/preprocess_points.cc
  lib/detector/point_pillars_detection/scatter_cuda.cu
  lib/ground_detector/ground_service_detector/ground_service_detector.cc
  lib/ground_detector/spatio_temporal_ground_detector/spatio_temporal_ground_detector.cc
  lib/map_manager/map_manager.cc
  lib/object_builder/object_builder.cc
  lib/object_filter_bank/object_filter_bank.cc
  lib/object_filter_bank/roi_boundary_filter/roi_boundary_filter.cc
  lib/pointcloud_preprocessor/pointcloud_preprocessor.cc
  lib/roi_filter/hdmap_roi_filter/bitmap2d.cc
  lib/roi_filter/hdmap_roi_filter/hdmap_roi_filter.cc
  lib/roi_filter/roi_service_filter/roi_service_filter.cc
  lib/scene_manager/ground_service/ground_service.cc
  lib/scene_manager/roi_service/roi_service.cc
  lib/scene_manager/scene_manager.cc
  lib/tracker/association/distance_collection.cc
  lib/tracker/association/gnn_bipartite_graph_matcher.cc
  lib/tracker/association/multi_hm_bipartite_graph_matcher.cc
  lib/tracker/common/mlf_track_data.cc
  lib/tracker/common/track_data.cc
  lib/tracker/common/tracked_object.cc
  lib/tracker/common/track_pool_types.cc
  lib/tracker/measurement/measurement_collection.cc
  # lib/tracker/multi_lidar_fusion/mlf_engine.cc
  lib/tracker/multi_lidar_fusion/mlf_motion_filter.cc
  lib/tracker/multi_lidar_fusion/mlf_motion_measurement.cc
  lib/tracker/multi_lidar_fusion/mlf_motion_refiner.cc
  lib/tracker/multi_lidar_fusion/mlf_shape_filter.cc
  lib/tracker/multi_lidar_fusion/mlf_tracker.cc
  lib/tracker/multi_lidar_fusion/mlf_track_object_distance.cc
  lib/tracker/multi_lidar_fusion/mlf_track_object_matcher.cc
  # lib/tracker/semantic_map/evaluator_manager.cc
  # tools
  # tools/exporter/msg_exporter.cc
  # tools/exporter/msg_exporter_main.cc
  # tools/offline_lidar_obstacle_perception.cc
)
target_link_libraries(perception_lidar 
  ${CUDA_LIBRARIES}
  ${TORCH_LIBRARIES}
  perception_proto
  perception_base
  perception_common
  perception_inference
  perception_lidar_app_proto
  perception_lidar_lib_detector_cnn_segmentation_proto
  perception_lidar_lib_detector_ncut_segmentation_proto
  perception_lidar_lib_ground_detector_ground_service_detector_proto
  perception_lidar_lib_ground_detector_spatio_temporal_ground_detector_proto
  perception_lidar_lib_object_filter_bank_proto
  perception_lidar_lib_pointcloud_preprocessor_proto
  perception_lidar_lib_roi_filter_hdmap_roi_filter_proto
  perception_lidar_lib_scene_manager_proto
  perception_lidar_lib_scene_manager_ground_service_proto
  perception_lidar_lib_scene_manager_roi_service_proto
  perception_lidar_lib_tracker_multi_lidar_fusion_proto
)
