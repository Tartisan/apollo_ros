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

#include "modules/tools/data_conversion/channel_info.h"

namespace apollo {
namespace data {

namespace {
static const char *empty_str_ = "";
} // namespace

ChannelInfo::ChannelInfo() { InitChannels(); }

ChannelInfo::~ChannelInfo() {
  channel_msg_type_.clear();
  channel_proto_desc_.clear();
}

const std::string ChannelInfo::GetMessageType(const std::string &channel_name) {
  auto search = channel_msg_type_.find(channel_name);
  if (search != channel_msg_type_.end()) {
    return search->second;
  }
  return empty_str_;
}

const std::string ChannelInfo::GetProtoDesc(const std::string &channel_name) {
  auto search = channel_proto_desc_.find(channel_name);
  if (search != channel_proto_desc_.end()) {
    return search->second;
  }
  return empty_str_;
}

const std::vector<std::string> &ChannelInfo::GetSupportChannels() {
  return support_channels_;
}

void ChannelInfo::InitChannels() {
  InitChannelInfo<apollo::localization::LocalizationEstimate>(
      "/apollo/localization/pose", "apollo.localization.LocalizationEstimate");
  InitChannelInfo<apollo::transform::TransformStampeds>(
      "/tf", "apollo.transform.TransformStampeds");
  InitChannelInfo<apollo::transform::TransformStampeds>(
      "/tf_static", "apollo.transform.TransformStampeds");
  InitChannelInfo<apollo::drivers::PointCloud>(
      "/apollo/sensor/velodyne64/compensator/PointCloud2",
      "apollo.drivers.PointCloud");
  InitChannelInfo<apollo::drivers::Image>(
      "/apollo/sensor/camera/front_6mm/image", "apollo.drivers.Image");
  InitChannelInfo<apollo::drivers::CompressedImage>(
      "/apollo/sensor/camera/front_6mm/image/compressed",
      "apollo.drivers.Image");
}

} // namespace data
} // namespace apollo
