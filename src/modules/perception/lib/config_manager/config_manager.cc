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
#include "modules/perception/lib/config_manager/config_manager.h"

#include "cyber/common/environment.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace lib {

using cyber::common::GetAbsolutePath;
using cyber::common::GetProtoFromASCIIFile;

ConfigManager::ConfigManager() {
  // work_root_ = FLAGS_work_root;

  // // For start at arbitrary path
  // if (work_root_.empty()) {
  //   work_root_ = cyber::common::GetEnv("MODULE_PATH");
  //   if (work_root_.empty()) {
  //     work_root_ = cyber::common::GetEnv("CYBER_PATH");
  //   }
  // }

  std::string cyber_package_path;
  FILE *pp = popen("rospack find cyber", "r"); //建立管道
  if (!pp) {
    AERROR << "Can't rospack find cyber";
    exit(1);
  }
  char tmp[1024]; //设置一个合适的长度，以存储每一行输出
  while (fgets(tmp, sizeof(tmp), pp) != NULL) {
    if (tmp[strlen(tmp) - 1] == '\n') {
      tmp[strlen(tmp) - 1] = '\0'; //去除换行符
    }
    cyber_package_path = tmp;
  }
  pclose(pp); //关闭管道
  work_root_ = cyber_package_path + "/../configs";
  InitConfigPath();
}

bool ConfigManager::Init() {
  MutexLock lock(&mutex_);
  return InitConfigPath();
}

bool ConfigManager::InitInternal() {
  if (inited_) {
    return true;
  }
  for (auto iter = model_config_map_.begin(); iter != model_config_map_.end();
       ++iter) {
    delete iter->second;
  }
  model_config_map_.clear();

  std::string config_module_path =
      GetAbsolutePath(work_root_, FLAGS_config_manager_path);
  AINFO << "WORK_ROOT: " << work_root_
        << " config_root_path: " << config_module_path;

  std::vector<std::string> model_config_files;
  if (!common::GetFileList(config_module_path, "config_manager.config",
                           &model_config_files)) {
    AERROR << "config_root_path : " << config_module_path
           << " get file list error.";
    return false;
  }

  for (const auto &model_config_file : model_config_files) {
    ModelConfigFileListProto file_list_proto;
    if (!GetProtoFromASCIIFile(model_config_file, &file_list_proto)) {
      AERROR << "Invalid ModelConfigFileListProto file: " << model_config_file;
      return false;
    }

    for (const std::string &model_config_path :
         file_list_proto.model_config_path()) {
      const std::string abs_path =
          GetAbsolutePath(work_root_, model_config_path);
      MultiModelConfigProto multi_model_config_proto;
      if (!GetProtoFromASCIIFile(abs_path, &multi_model_config_proto)) {
        AERROR << "Invalid MultiModelConfigProto file: " << abs_path;
        return false;
      }

      for (const ModelConfigProto &model_config_proto :
           multi_model_config_proto.model_configs()) {
        ModelConfig *model_config = new ModelConfig();
        if (!model_config->Reset(model_config_proto)) {
          return false;
        }

        AINFO << "load ModelConfig succ. name: " << model_config->name();

        auto result =
            model_config_map_.emplace(model_config->name(), model_config);
        if (!result.second) {
          AWARN << "duplicate ModelConfig, name: " << model_config->name();
          return false;
        }
      }
    }
  }

  // AINFO << "finish to load ModelConfigs. NumModels: "
  //       << model_config_map_.size();

  inited_ = true;

  return true;
}

bool ConfigManager::Reset() {
  MutexLock lock(&mutex_);
  inited_ = false;
  return InitConfigPath();
}

bool ConfigManager::GetModelConfig(const std::string &model_name,
                                   const ModelConfig **model_config) {
  // if (!inited_ && !Init()) {
  //   return false;
  // }

  auto citer = model_config_map_.find(model_name);
  if (citer == model_config_map_.end()) {
    return false;
  }
  *model_config = citer->second;
  return true;
}

bool ConfigManager::LodeModelConfigFile(const std::string &config_file) {
  if (!cyber::common::PathExists(config_file)) {
    AERROR << "file not exit: " << config_file;
    return false;
  }
  for (auto iter = model_config_map_.begin(); iter != model_config_map_.end();
       ++iter) {
    delete iter->second;
  }
  model_config_map_.clear();

  MultiModelConfigProto multi_model_config_proto;
  if (!GetProtoFromASCIIFile(config_file, &multi_model_config_proto)) {
    AERROR << "Invalid MultiModelConfigProto file: " << config_file;
    return false;
  }

  for (const ModelConfigProto &model_config_proto :
       multi_model_config_proto.model_configs()) {
    ModelConfig *model_config = new ModelConfig();
    if (!model_config->Reset(model_config_proto)) {
      return false;
    }

    AINFO << "load ModelConfig succ. name: " << model_config->name();

    auto result = model_config_map_.emplace(model_config->name(), model_config);
    if (!result.second) {
      AWARN << "duplicate ModelConfig, name: " << model_config->name();
      return false;
    }
  }

  return true;
}

bool ConfigManager::InitConfigPath() {
  if (inited_) {
    return true;
  }
  if (!LodeModelConfigFile(work_root_ + "/car_select.conf")) {
    return false;
  }
  const ModelConfig *model_config = nullptr;
  GetModelConfig("car_select", &model_config);
  std::string current_carname;
  if (!model_config->get_value("current_carname", &current_carname)) {
    AERROR << "cannot get param current_carname";
    return false;
  }
  config_path_ = work_root_ + "/" + current_carname;
  inited_ = true;
  return true;
}

ConfigManager::~ConfigManager() {
  for (auto iter = model_config_map_.begin(); iter != model_config_map_.end();
       ++iter) {
    delete iter->second;
  }
}

bool ModelConfig::Reset(const ModelConfigProto &proto) {
  name_ = proto.name();
  version_ = proto.version();

  integer_param_map_.clear();
  string_param_map_.clear();
  double_param_map_.clear();
  float_param_map_.clear();
  bool_param_map_.clear();
  array_integer_param_map_.clear();
  array_string_param_map_.clear();
  array_double_param_map_.clear();
  array_float_param_map_.clear();
  array_bool_param_map_.clear();

  for (const KeyValueInt &pair : proto.integer_params()) {
    integer_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueString &pair : proto.string_params()) {
    string_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueDouble &pair : proto.double_params()) {
    double_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueFloat &pair : proto.float_params()) {
    float_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueBool &pair : proto.bool_params()) {
    bool_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueArrayInt &pair : proto.array_integer_params()) {
    std::vector<int> values;
    RepeatedToVector(pair.values(), &values);
    array_integer_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayString &pair : proto.array_string_params()) {
    std::vector<std::string> values;
    values.reserve(pair.values_size());
    for (const std::string &value : pair.values()) {
      values.push_back(value);
    }
    array_string_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayDouble &pair : proto.array_double_params()) {
    std::vector<double> values;
    RepeatedToVector(pair.values(), &values);
    array_double_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayFloat &pair : proto.array_float_params()) {
    std::vector<float> values;
    RepeatedToVector(pair.values(), &values);
    array_float_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayBool &pair : proto.array_bool_params()) {
    std::vector<bool> values;
    RepeatedToVector(pair.values(), &values);
    array_bool_param_map_.emplace(pair.name(), values);
  }

  AINFO << "reset ModelConfig. model_name: " << name_
        << " integer_param_map's size: " << integer_param_map_.size()
        << " string_param_map's size: " << string_param_map_.size()
        << " double_param_map's size: " << double_param_map_.size()
        << " float_param_map's size: " << float_param_map_.size()
        << " bool_param_map's size: " << bool_param_map_.size()
        << " array_integer_param_map's size: "
        << array_integer_param_map_.size()
        << " array_string_param_map's size: " << array_string_param_map_.size()
        << " array_double_param_map's size: " << array_double_param_map_.size()
        << " array_float_param_map's size: " << array_float_param_map_.size()
        << " array_bool_param_map's size: " << array_bool_param_map_.size();

  return true;
}

} // namespace lib
} // namespace perception
} // namespace apollo
