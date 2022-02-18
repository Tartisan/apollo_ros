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
#include "cyber/record/record_viewer.h"

#include "modules/tools/data_conversion/lib/convert.h"

using apollo::cyber::record::RecordMessage;
using apollo::cyber::record::RecordReader;
using apollo::cyber::record::RecordViewer;

void PrintUsage() {
  std::cout << "Usage:\n"
            << "  record_to_rosbag input.record output.bag" << std::endl;
}

int main(int argc, char **argv) {
  if (argc != 3) {
    PrintUsage();
    return -1;
  }

  const std::string record_file_name = argv[1];
  const std::string rosbag_file_name = argv[2];

  rosbag::Bag bag;
  bag.open(rosbag_file_name, rosbag::bagmode::Write);

  ros::Time::init();
  ros::Time start_time = ros::Time::now();

  auto reader = std::make_shared<RecordReader>(record_file_name);
  RecordViewer viewer(reader);
  for (const RecordMessage &m : viewer) {
    std::string msg_name = m.channel_name;
    uint64_t nsec = m.time;

    ros::Time t;
    t.fromNSec(nsec);

    if (msg_name == "/tf" || msg_name == "/tf_static") {
      apollo::transform::TransformStampeds pb;
      tf2_msgs::TFMessage msg;
      pb.ParseFromString(m.content);
      ConvertTransformStampedsFromPbToRos(&pb, &msg);
      bag.write(msg_name, t, msg);
    } else if (msg_name == "/apollo/localization/pose") {
      apollo::localization::LocalizationEstimate pb;
      localization::LocalizationEstimate msg;
      pb.ParseFromString(m.content);
      ConvertLocalizationEstimateFromPbToRos(&pb, &msg);
      bag.write(msg_name, t, msg);
    } else if (msg_name.find("image/compressed") != std::string::npos) {
      apollo::drivers::CompressedImage pb;
      sensor_msgs::CompressedImage msg;
      pb.ParseFromString(m.content);
      ConvertCompressedImageFromPbToRos(&pb, &msg);
      bag.write(msg_name, t, msg);
    } else if (msg_name.find("image") != std::string::npos && 
               msg_name.find("compressed") == std::string::npos) {
      apollo::drivers::Image pb;
      sensor_msgs::Image msg;
      pb.ParseFromString(m.content);
      ConvertImageFromPbToRos(&pb, &msg);
      bag.write(msg_name, t, msg);
    } else if (msg_name.find("PointCloud2") != std::string::npos) {
      apollo::drivers::PointCloud pb;
      sensor_msgs::PointCloud2 msg;
      pb.ParseFromString(m.content);
      ConvertPointCloudFromPbToRos(&pb, &msg);
      bag.write(msg_name, t, msg);
    } else if (msg_name.find("/apollo/sensor/radar") != std::string::npos) {
      apollo::drivers::ContiRadar pb;
      drivers::ContiRadar msg;
      pb.ParseFromString(m.content);
      ConvertContiRadarFromPbToRos(&pb, &msg);
      bag.write(msg_name, t, msg);
    } else {
      AWARN << "not support channel:" << msg_name;
      continue;
    }
  }

  bag.close();

  std::cout << "Info of rosbag file" << std::endl;
  std::string command_line = "rosbag info " + rosbag_file_name;
  int res = system(command_line.c_str());

  std::cout << "Convertion finished! Took " << ros::Time::now() - start_time
            << "s in total." << std::endl;
  return res;
}
