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

#ifndef CYBER_COMMON_ENVIRONMENT_H_
#define CYBER_COMMON_ENVIRONMENT_H_

#include <cassert>
#include <string>

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace common {

inline std::string GetEnv(const std::string& var_name,
                          const std::string& default_value = "") {
  const char* var = std::getenv(var_name.c_str());
  if (var == nullptr) {
    AWARN << "Environment variable [" << var_name << "] not set, fallback to "
          << default_value;
    return default_value;
  }
  return std::string(var);
}

inline const std::string WorkRoot() {
  std::string cyber_pkg_path = "";
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
    cyber_pkg_path = tmp;
  }
  pclose(pp); //关闭管道

  // std::string work_root = GetEnv("CYBER_PATH", cyber_pkg_path);
  std::string work_root;
  if (work_root.empty()) {
    work_root = cyber_pkg_path;
  }
  return work_root;
}

}  // namespace common
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_COMMON_ENVIRONMENT_H_
