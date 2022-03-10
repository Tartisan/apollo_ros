/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/onboard/component/detection_component.h"

#include "cyber/time/clock.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/lidar/common/lidar_error_code.h"
#include "modules/perception/lidar/common/lidar_frame_pool.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/common/lidar_timer.h"
#include "modules/perception/onboard/common_flags/common_flags.h"

#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>

using ::apollo::cyber::Clock;

namespace apollo {
namespace perception {
namespace onboard {

std::atomic<uint32_t> DetectionComponent::seq_num_{0};

bool DetectionComponent::Init(ros::NodeHandle nh, ros::NodeHandle private_nh) {
  // ros params
  std::string comp_file;
  std::string input_channel;
  private_nh.param(
      "comp_file", comp_file,
      std::string("/perception/conf/lidar/velodyne64_detection_conf.pb.txt"));
  private_nh.param(
      "channel", input_channel,
      std::string("/apollo/sensor/velodyne64/compensator/PointCloud2"));
  LidarDetectionComponentConfig comp_config;
  // if (!GetProtoConfig(&comp_config)) {
  //   AERROR << "Get config failed";
  //   return false;
  // }
  std::string config_path = cyber::common::GetConfigPath();
  if (!cyber::common::GetProtoFromFile(config_path + comp_file, &comp_config)) {
    AERROR << "Get config failed: " << config_path + comp_file;
    return false;
  }
  AINFO << "Lidar Component Configs: " << comp_config.DebugString();
  output_channel_name_ = comp_config.output_channel_name();
  sensor_name_ = comp_config.sensor_name();
  detector_name_ = comp_config.detector_name();
  lidar2novatel_tf2_child_frame_id_ =
      comp_config.lidar2novatel_tf2_child_frame_id();
  lidar_query_tf_offset_ =
      static_cast<float>(comp_config.lidar_query_tf_offset());
  enable_hdmap_ = comp_config.enable_hdmap();
  enable_visualize_ = comp_config.enable_visualize();
  // writer_ = node_->CreateWriter<LidarFrameMessage>(output_channel_name_);

  if (enable_visualize_) {
    pub_segmented_objects_ =
        nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(
            "/perception/lidar_frame/segmented_objects", 1);
    pub_objects_polygon_ = nh.advertise<visualization_msgs::MarkerArray>(
        "/perception/lidar_frame/polygon", 1);
  }

  if (!InitAlgorithmPlugin()) {
    AERROR << "Failed to init detection_component algorithm plugin.";
    return false;
  }
  return true;
}

bool DetectionComponent::Proc(
    const sensor_msgs::PointCloud2::ConstPtr &ros_msg,
    const std::shared_ptr<LidarFrameMessage> &out_message) {
  AINFO << "=============================";
  AINFO << std::setprecision(16)
        << "Enter detection_component, message timestamp: "
        << ros_msg->header.stamp.toSec()
        << " current timestamp: " << Clock::NowInSeconds();
  lidar::Timer timer;
  auto pb_msg = std::make_shared<apollo::drivers::PointCloud>();
  ConvertPointCloudFromRosToPb(ros_msg, pb_msg);
  double ros2pb_time = timer.toc(true);
  AINFO << "ConvertPointCloudFromRosToPb time_cost: " << ros2pb_time;
  // auto out_message = std::make_shared<LidarFrameMessage>();

  bool status = InternalProc(pb_msg, out_message);
  // if (status) {
  //   writer_->Write(out_message);
  //   AINFO << "Send lidar detect output message.";
  // }
  AINFO << std::setprecision(16)
        << "Leave detection_component, message timestamp: "
        << ros_msg->header.stamp.toSec()
        << " current timestamp: " << Clock::NowInSeconds();
  double lidar_det_total_time = timer.toc(true);
  AINFO << "detection_component time_cost: " << lidar_det_total_time;
  return status;
}

bool DetectionComponent::InitAlgorithmPlugin() {
  ACHECK(common::SensorManager::Instance()->GetSensorInfo(sensor_name_,
                                                          &sensor_info_));

  lidar::BaseLidarObstacleDetection *detector =
      lidar::BaseLidarObstacleDetectionRegisterer::GetInstanceByName(
          detector_name_);
  CHECK_NOTNULL(detector);
  detector_.reset(detector);
  lidar::LidarObstacleDetectionInitOptions init_options;
  init_options.sensor_name = sensor_name_;
  init_options.enable_hdmap_input =
      FLAGS_obs_enable_hdmap_input && enable_hdmap_;
  ACHECK(detector_->Init(init_options))
      << "lidar obstacle detection init error";

  lidar2world_trans_.Init(lidar2novatel_tf2_child_frame_id_);
  return true;
}

bool DetectionComponent::InternalProc(
    const std::shared_ptr<const drivers::PointCloud> &in_message,
    const std::shared_ptr<LidarFrameMessage> &out_message) {
  uint32_t seq_num = seq_num_.fetch_add(1);
  const double timestamp = in_message->measurement_time();
  const double cur_time = Clock::NowInSeconds();
  const double start_latency = (cur_time - timestamp) * 1e3;
  AINFO << "FRAME_STATISTICS:Lidar:Start:msg_time[" << std::setprecision(19)
        << timestamp << "]:sensor[" << sensor_name_ << "]:cur_time[" << cur_time
        << "]:cur_latency[" << start_latency << "]";

  out_message->timestamp_ = timestamp;
  out_message->lidar_timestamp_ = in_message->header().lidar_timestamp();
  out_message->seq_num_ = seq_num;
  out_message->process_stage_ = ProcessStage::LIDAR_DETECTION;
  out_message->error_code_ = apollo::common::ErrorCode::OK;

  auto &frame = out_message->lidar_frame_;
  frame = lidar::LidarFramePool::Instance().Get();
  frame->cloud = base::PointFCloudPool::Instance().Get();
  frame->timestamp = timestamp;
  frame->sensor_info = sensor_info_;
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  Eigen::Affine3d pose_novatel = Eigen::Affine3d::Identity();
  const double lidar_query_tf_timestamp =
      timestamp - lidar_query_tf_offset_ * 0.001;
  // if (!lidar2world_trans_.GetSensor2worldTrans(lidar_query_tf_timestamp, &pose,
  //                                              &pose_novatel)) {
  //   out_message->error_code_ = apollo::common::ErrorCode::PERCEPTION_ERROR_TF;
  //   AERROR << "Failed to get pose at time: " << std::setprecision(19)
  //          << lidar_query_tf_timestamp;
  //   return false;
  // }

  frame->lidar2world_pose = pose;
  frame->novatel2world_pose = pose_novatel;

  lidar::LidarObstacleDetectionOptions detect_opts;
  detect_opts.sensor_name = sensor_name_;
  // lidar2world_trans_.GetExtrinsics(&detect_opts.sensor2novatel_extrinsics);
  lidar::LidarProcessResult ret =
      detector_->Process(detect_opts, in_message, frame.get());
  if (ret.error_code != lidar::LidarErrorCode::Succeed) {
    out_message->error_code_ =
        apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
    AERROR << "Lidar detection process error, " << ret.log;
    return false;
  }

  if (enable_visualize_) {
    VisualizeLidarFrame(in_message->header(), frame.get());
  }

  return true;
}

void DetectionComponent::VisualizeLidarFrame(
    const apollo::common::Header &header, lidar::LidarFrame *frame) {
  // segmented_objects boundingbox and polygon 
  jsk_recognition_msgs::BoundingBoxArray bounding_boxes;
  bounding_boxes.header.frame_id = header.frame_id();
  bounding_boxes.header.stamp = ros::Time(header.timestamp_sec());
  jsk_recognition_msgs::BoundingBox bounding_box;
  bounding_box.header.frame_id = header.frame_id();
  bounding_box.header.stamp = ros::Time(header.timestamp_sec());
  visualization_msgs::MarkerArray polygons;
  visualization_msgs::Marker polygon;
  polygon.header.frame_id = header.frame_id();
  polygon.header.stamp = ros::Time(header.timestamp_sec());
  polygon.type = visualization_msgs::Marker::LINE_STRIP;
  polygon.action = visualization_msgs::Marker::ADD;
  polygon.lifetime = ros::Duration(0.1);
  int object_count = 0;
  for (const auto &segment_object : frame->segmented_objects) {
    bounding_box.pose.position.x = segment_object->center[0];
    bounding_box.pose.position.y = segment_object->center[1];
    bounding_box.pose.position.z = segment_object->center[2];
    bounding_box.dimensions.x = segment_object->size[0];
    bounding_box.dimensions.y = segment_object->size[1];
    bounding_box.dimensions.z = segment_object->size[2];
    bounding_box.label = static_cast<uint32_t>(segment_object->type);
    geometry_msgs::Quaternion quat =
        tf::createQuaternionMsgFromYaw(segment_object->theta);
    bounding_box.pose.orientation = quat;
    bounding_boxes.boxes.push_back(bounding_box);

    polygon.id = object_count++;
    polygon.scale.x = 0.05;
    polygon.color.r = 1;
    polygon.color.g = 1;
    polygon.color.b = 1;
    polygon.color.a = 1;
    polygon.pose.orientation.w = 1.0;
    polygon.points.clear();
    geometry_msgs::Point point;
    int polygon_pts_size = segment_object->polygon.size();
    for (int i = 0; i <= polygon_pts_size; ++i) {
      const auto &p = segment_object->polygon.at(i % polygon_pts_size);
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      polygon.points.push_back(point);
    }
    polygons.markers.push_back(polygon);
  }
  pub_segmented_objects_.publish(bounding_boxes);
  pub_objects_polygon_.publish(polygons);
}

} // namespace onboard
} // namespace perception
} // namespace apollo
