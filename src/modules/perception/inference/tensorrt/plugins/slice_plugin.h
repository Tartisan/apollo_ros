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

#include <algorithm>
#include <vector>

#include "modules/perception/inference/tensorrt/rt_common.h"

namespace apollo {
namespace perception {
namespace inference {

class SLICEPlugin : public nvinfer1::IPluginV2 {
 public:
  SLICEPlugin(const SliceParameter &param, const nvinfer1::Dims &in_dims) {
    CHECK_GT(param.slice_point_size(), 0);
    for (int i = 0; i < param.slice_point_size(); i++) {
      slice_point_.push_back(param.slice_point(i));
    }
    axis_ = std::max(param.axis() - 1, 0);
    input_dims_.nbDims = in_dims.nbDims;
    CHECK_GT(input_dims_.nbDims, 0);
    for (int i = 0; i < in_dims.nbDims; i++) {
      input_dims_.d[i] = in_dims.d[i];
      // input_dims_.type[i] = in_dims.type[i];
    }

    for (size_t i = 0; i < slice_point_.size(); i++) {
      if (i == 0) {
        out_slice_dims_.push_back(slice_point_[i]);
      } else {
        out_slice_dims_.push_back(slice_point_[i] - slice_point_[i - 1]);
      }
    }
    out_slice_dims_.push_back(input_dims_.d[axis_] -
                              slice_point_[slice_point_.size() - 1]);
  }
  SLICEPlugin() {}
  ~SLICEPlugin() {}
  virtual int initialize() noexcept override { return 0; }
  virtual void terminate() noexcept override {}
  int getNbOutputs() const noexcept override {
    return static_cast<int>(slice_point_.size()) + 1;
  }
  nvinfer1::Dims getOutputDimensions(int index, const nvinfer1::Dims *inputs,
                                     int nbInputDims) noexcept override {
    nvinfer1::Dims out_dims = inputs[0];
    out_dims.d[axis_] = out_slice_dims_[index];
    return out_dims;
  }

  void configureWithFormat(const nvinfer1::Dims *inputDims,
                           int nbInputs,
                           const nvinfer1::Dims *outputDims,
                           int nbOutputs,
                           nvinfer1::DataType type, 
                           nvinfer1::PluginFormat format, 
                           int maxBatchSize) noexcept override {
    input_dims_ = inputDims[0];
  }

  size_t getWorkspaceSize(int maxBatchSize) const noexcept override { return 0; }

  int enqueue(int batchSize, void const* const* inputs, void* const* outputs, 
              void* workspace, cudaStream_t stream) noexcept override;

  size_t getSerializationSize() const noexcept override { return 0; }

  void serialize(void *buffer) const noexcept override {
    char *p = reinterpret_cast<char *>(buffer);
    auto ref = p;
    // setPluginType(&p, "Slice");

    reinterpret_cast<int *>(p)[0] = input_dims_.nbDims;
    p += sizeof(int);
    for (int i = 0; i < input_dims_.nbDims; ++i) {
      reinterpret_cast<int *>(p)[0] = input_dims_.d[i];
      p += sizeof(int);
      // reinterpret_cast<int *>(p)[0] = static_cast<int>(input_dims_.type[i]);
      // p += sizeof(int);
    }

    reinterpret_cast<size_t *>(p)[0] = slice_point_.size();
    p += sizeof(size_t);
    for (size_t i = 0; i < slice_point_.size(); ++i) {
      reinterpret_cast<int *>(p)[0] = slice_point_[i];
      p += sizeof(int);
    }

    reinterpret_cast<size_t *>(p)[0] = out_slice_dims_.size();
    p += sizeof(size_t);
    for (size_t i = 0; i < out_slice_dims_.size(); ++i) {
      reinterpret_cast<int *>(p)[0] = out_slice_dims_[i];
      p += sizeof(int);
    }

    reinterpret_cast<int *>(p)[0] = axis_;
    p += sizeof(int);

    CHECK(p - ref == getSerializationSize());
  }

  char const* getPluginType() const noexcept override { return "Slice"; }
  
  char const* getPluginVersion() const noexcept override { return "1"; }

  bool supportsFormat(nvinfer1::DataType type, nvinfer1::PluginFormat format) const noexcept override {
    return true;
  }

  void destroy() noexcept override {}

  nvinfer1::IPluginV2* clone() const noexcept override {
    return nullptr;
  }

  void setPluginNamespace(char const* pluginNamespace) noexcept override {}

  char const* getPluginNamespace() const noexcept override { return "inference"; }

 private:
  std::vector<int> slice_point_;
  std::vector<int> out_slice_dims_;
  int axis_;
  nvinfer1::Dims input_dims_;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
