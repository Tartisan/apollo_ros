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

#include "modules/transform/static_transform_component.h"
#include "cyber/common/environment.h"
#include "cyber/common/file.h"
#include "yaml-cpp/yaml.h"

namespace apollo {
namespace transform {

bool StaticTransformComponent::Init() {
  car_path_ = cyber::common::GetConfigPath();
  std::string config_file_path =
      car_path_ + "/transform/conf/static_transform_conf.pb.txt";
  if (!cyber::common::GetProtoFromFile(config_file_path, &conf_)) {
    AERROR << "Parse conf file failed, " << config_file_path;
    return false;
  }
  AINFO << "The static_transform conf file is loaded: " << config_file_path;

  SendTransforms();
  return true;
}

void StaticTransformComponent::SendTransforms() {
  for (auto &extrinsic_file : conf_.extrinsic_file()) {
    if (extrinsic_file.enable()) {
      AINFO << "Broadcast static transform, frame id ["
            << extrinsic_file.frame_id() << "], child frame id ["
            << extrinsic_file.child_frame_id() << "]";
      SendTransform(car_path_ + extrinsic_file.file_path());
    }
  }
}

bool StaticTransformComponent::SendTransform(const std::string &file_path) {
  if (!cyber::common::PathExists(file_path)) {
    AERROR << "Extrinsic yaml file is noe exists: " << file_path;
    return false;
  }
  YAML::Node tf = YAML::LoadFile(file_path);
  try {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id =
        tf["header"]["frame_id"].as<std::string>();
    transform_stamped.child_frame_id = tf["child_frame_id"].as<std::string>();
    // translatiion
    geometry_msgs::Vector3 translation;
    translation.x = tf["transform"]["translation"]["x"].as<double>();
    translation.y = tf["transform"]["translation"]["y"].as<double>();
    translation.z = tf["transform"]["translation"]["z"].as<double>();
    // rotation
    geometry_msgs::Quaternion rotation;
    rotation.x = tf["transform"]["rotation"]["x"].as<double>();
    rotation.y = tf["transform"]["rotation"]["y"].as<double>();
    rotation.z = tf["transform"]["rotation"]["z"].as<double>();
    rotation.w = tf["transform"]["rotation"]["w"].as<double>();

    transform_stamped.transform.translation = translation;
    transform_stamped.transform.rotation = rotation;
    static_broadcaster.sendTransform(transform_stamped);
  } catch (...) {
    AERROR << "Extrinsic yaml file parse failed: " << file_path;
    return false;
  }
  return true;
}

} // namespace transform
} // namespace apollo
