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
#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "cyber/cyber.h"

#include "modules/tools/data_conversion/lib/convert.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/perception/lidar/app/lidar_obstacle_detection.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/onboard/component/lidar_inner_component_messages.h"
#include "modules/perception/onboard/proto/lidar_component_config.pb.h"
#include "modules/perception/onboard/transform_wrapper/transform_wrapper.h"

namespace apollo {
namespace perception {
namespace onboard {

class DetectionComponent {
 public:
  DetectionComponent() {}
  ~DetectionComponent() {}

  bool Init(ros::NodeHandle nh, ros::NodeHandle private_nh);
  bool Proc(const sensor_msgs::PointCloud2::ConstPtr& ros_msg, 
            const std::shared_ptr<LidarFrameMessage> &out_message);
  void VisualizeLidarFrame(const apollo::common::Header &header, 
                           lidar::LidarFrame *frame);

 private:
  bool InitAlgorithmPlugin();
  bool InternalProc(
      const std::shared_ptr<const drivers::PointCloud>& in_message,
      const std::shared_ptr<LidarFrameMessage>& out_message);

 private:
  static std::atomic<uint32_t> seq_num_;
  std::string sensor_name_;
  std::string detector_name_;
  bool enable_hdmap_ = true;
  bool enable_visualize_ = false;
  float lidar_query_tf_offset_ = 20.0f;
  std::string lidar2novatel_tf2_child_frame_id_;
  std::string output_channel_name_;
  base::SensorInfo sensor_info_;
  TransformWrapper lidar2world_trans_;
  std::unique_ptr<lidar::BaseLidarObstacleDetection> detector_;
  
  ros::Publisher pub_segmented_objects_;
  ros::Publisher pub_non_ground_points_;
  ros::Publisher pub_ground_points_;
  // std::shared_ptr<apollo::cyber::Writer<LidarFrameMessage>> writer_;
};

// CYBER_REGISTER_COMPONENT(DetectionComponent);

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
