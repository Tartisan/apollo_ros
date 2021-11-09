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

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "cyber/message/raw_message.h"
#include "cyber/proto/record.pb.h"
#include "cyber/record/file/record_file_writer.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"

#include "modules/tools/data_conversion/channel_info.h"
#include "modules/tools/data_conversion/convert.h"

using apollo::cyber::proto::SingleMessage;
using apollo::data::ChannelInfo;

void PrintUsage() {
  std::cout << "Usage:\n"
            << "  rosbag_to_record input.bag output.record" << std::endl;
}

int main(int argc, char **argv) {
  if (argc != 3) {
    PrintUsage();
    return -1;
  }

  const std::string rosbag_file_name = argv[1];
  const std::string record_file_name = argv[2];
  rosbag::Bag bag;
  try {
    bag.open(rosbag_file_name);
  } catch (...) {
    std::cerr << "Error: the input file is not a ros bag file." << std::endl;
    return -1;
  }
  auto channel_info = ChannelInfo::Instance();

  auto record_writer = std::make_shared<apollo::cyber::record::RecordWriter>();
  record_writer->SetSizeOfFileSegmentation(0);
  record_writer->SetIntervalOfFileSegmentation(0);
  if (!record_writer->Open(record_file_name)) {
    std::cerr << "Error: open file[" << record_file_name << "] failed.";
  }

  ros::Time::init();
  ros::Time start_time = ros::Time::now();

  rosbag::View view(bag,
                    rosbag::TopicQuery(channel_info->GetSupportChannels()));

  std::vector<std::string> channel_write_flag;
  for (const rosbag::MessageInstance m : view) {
    const std::string msg_type = m.getDataType();
    const std::string channel_name = m.getTopic();

    auto desc = channel_info->GetProtoDesc(channel_name);
    auto record_message_type = channel_info->GetMessageType(channel_name);
    if (desc == "" || record_message_type == "") {
      AWARN << "can not find desc or message type for channel: " << channel_name
            << "; desc:" << desc << ";type:" << record_message_type;
    }

    apollo::cyber::proto::Channel channel;
    channel.set_name(channel_name);
    channel.set_message_type(record_message_type);
    channel.set_proto_desc(desc);
    if (std::find(channel_write_flag.begin(), channel_write_flag.end(),
                  channel_name) == channel_write_flag.end() &&
        !record_writer->WriteChannel(channel_name, record_message_type, desc)) {
      AERROR << "write channel info failed";
    } else {
      channel_write_flag.push_back(channel_name);
    }
  }

  for (const rosbag::MessageInstance m : view) {
    const std::string msg_type = m.getDataType();
    const std::string channel_name = m.getTopic();
    uint64_t nsec = m.getTime().toNSec();

    // FIXME: find a way get all serialized str;
    std::string serialized_str;
    if (channel_name == "/apollo/sensor/velodyne64/compensator/PointCloud2") {
      auto msg = m.instantiate<sensor_msgs::PointCloud2>();
      auto pb = std::make_shared<apollo::drivers::PointCloud>();
      ConvertPointCloudFromRosToPb(msg, pb);
      pb->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/sensor/camera/front_6mm/image") {
      auto msg = m.instantiate<sensor_msgs::Image>();
      auto pb = std::make_shared<apollo::drivers::Image>();
      ConvertImageFromRosToPb(msg, pb);
      pb->SerializeToString(&serialized_str);
    } else if (channel_name == "/tf" || channel_name == "/tf_static") {
      auto msg = m.instantiate<tf2_msgs::TFMessage>();
      auto pb = std::make_shared<apollo::transform::TransformStampeds>();
      ConvertTransformStampedsFromRosToPb(msg, pb);
      pb->SerializeToString(&serialized_str);
    } else {
      AWARN << "not support channel:" << channel_name;
      continue;
    }

    auto raw_msg =
        std::make_shared<apollo::cyber::message::RawMessage>(serialized_str);
    if (!record_writer->WriteMessage(channel_name, raw_msg, nsec)) {
      AERROR << "write single msg fail";
    }
  }

  record_writer->Close();
  record_writer = nullptr;
  std::cout << "Info of record file" << std::endl;
  std::string command_line = "cyber_recorder info " + record_file_name;
  int res = system(command_line.c_str());

  std::cout << "Convertion finished! Took " << ros::Time::now() - start_time
            << " seconds in total." << std::endl;
  return res;
}
