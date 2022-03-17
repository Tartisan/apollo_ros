/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

class ReLUPlugin : public nvinfer1::IPluginV2 {
 public:
  ReLUPlugin(const ReLUParameter &param, const nvinfer1::Dims &in_dims) {
    input_dims_.nbDims = in_dims.nbDims;
    CHECK_GT(input_dims_.nbDims, 0);
    for (int i = 0; i < in_dims.nbDims; i++) {
      input_dims_.d[i] = in_dims.d[i];
      // input_dims_.type[i] = in_dims.type[i];
    }
    negative_slope_ = param.negative_slope();
  }

  ReLUPlugin() {}
  ~ReLUPlugin() {}
  virtual int initialize() noexcept override { return 0; }
  virtual void terminate() noexcept override {}
  int getNbOutputs() const noexcept override { return 1; }

  nvinfer1::Dims getOutputDimensions(int index, const nvinfer1::Dims *inputs,
                                     int nbInputDims) noexcept override {
    nvinfer1::Dims out_dims = inputs[0];
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
    char *d = reinterpret_cast<char *>(buffer), *a = d;
    size_t size = getSerializationSize();
    CHECK_EQ(d, a + size);
  }

  char const* getPluginType() const noexcept override { return "ReLU"; }
  
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
  float negative_slope_;
  nvinfer1::Dims input_dims_;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
