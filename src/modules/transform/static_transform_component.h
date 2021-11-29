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
#include <vector>

// #include "cyber/component/component.h"
#include "modules/transform/proto/static_transform_conf.pb.h"
// #include "modules/transform/proto/transform.pb.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace apollo {
namespace transform {

class StaticTransformComponent {
public:
  StaticTransformComponent() {}
  ~StaticTransformComponent() {}

public:
  bool Init();

private:
  void SendTransforms();
  bool SendTransform(const std::string &file_path);

  std::string car_path_;
  apollo::static_transform::Conf conf_;
};

// CYBER_REGISTER_COMPONENT(StaticTransformComponent)

} // namespace transform
} // namespace apollo
