/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/message/message_traits.h"
#include "cyber/message/protobuf_factory.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/transform/proto/transform.pb.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <tf2_msgs/TFMessage.h>

#include "drivers/ContiRadar.h"
#include "localization/LocalizationEstimate.h"

// header
void ConvertHeaderFromRosToPb(const std_msgs::Header *msg, 
                              apollo::common::Header *pb);
void ConvertHeaderFromPbToRos(const apollo::common::Header *pb,
                              std_msgs::Header *msg);
// pointcloud
void ConvertPointCloudFromRosToPb(
    const sensor_msgs::PointCloud2::ConstPtr &msg,
    std::shared_ptr<apollo::drivers::PointCloud> &pb);
void ConvertPointCloudFromPbToRos(
    apollo::drivers::PointCloud *pb, sensor_msgs::PointCloud2 *msg);
// image
void ConvertImageFromRosToPb(const sensor_msgs::Image::ConstPtr &msg,
                             std::shared_ptr<apollo::drivers::Image> &pb);
void ConvertImageFromPbToRos(apollo::drivers::Image *pb,
                             sensor_msgs::Image *msg);
// compressed image
void ConvertCompressedImageFromRosToPb(
    const sensor_msgs::CompressedImage::ConstPtr &msg, 
    std::shared_ptr<apollo::drivers::CompressedImage> &pb);
void ConvertCompressedImageFromPbToRos(apollo::drivers::CompressedImage *pb,
                                       sensor_msgs::CompressedImage *msg);
// conti_radar
void ConvertContiRadarFromPbToRos(apollo::drivers::ContiRadar *pb,
                                  drivers::ContiRadar *msg);
// transform
void ConvertTransformFromPbToRos(apollo::transform::Transform *pb,
                                 geometry_msgs::Transform *msg);
// transform stampeds
void ConvertTransformStampedsFromRosToPb(
    const tf2_msgs::TFMessageConstPtr &msg,
    std::shared_ptr<apollo::transform::TransformStampeds> &pb);
void ConvertTransformStampedsFromPbToRos(
    apollo::transform::TransformStampeds *pb, tf2_msgs::TFMessage *msg);
// point
template <typename PointType>
void ConvertPointFromPbToRos(PointType *pb, geometry_msgs::Point *msg);
// quaternion
void ConvertQuaternionFromPbToRos(apollo::common::Quaternion *pb,
                                  geometry_msgs::Quaternion *msg);
// localization
void ConvertLocalizationEstimateFromPbToRos(
    apollo::localization::LocalizationEstimate *pb,
    localization::LocalizationEstimate *msg);

