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

#include <limits>

#include "modules/perception/proto/rt.pb.h"

#include "modules/perception/inference/tensorrt/rt_common.h"

namespace apollo {
namespace perception {
namespace inference {

class ArgMax1Plugin : public nvinfer1::IPluginV2 {
 public:
  ArgMax1Plugin(const ArgMaxParameter &argmax_param, nvinfer1::Dims in_dims)
      : float_min_(std::numeric_limits<float>::min()) {
    input_dims_.nbDims = in_dims.nbDims;
    CHECK_GT(input_dims_.nbDims, 0);
    for (int i = 0; i < in_dims.nbDims; i++) {
      input_dims_.d[i] = in_dims.d[i];
      // input_dims_.type[i] = in_dims.type[i];
    }
    axis_ = argmax_param.axis();
    out_max_val_ = argmax_param.out_max_val();
    top_k_ = argmax_param.top_k();
    CHECK_GE(top_k_, static_cast<size_t>(1))
        << "top k must not be less than 1.";
    output_dims_ = input_dims_;
    output_dims_.d[0] = 1;
    if (out_max_val_) {
      // Produces max_ind and max_val
      output_dims_.d[0] = 2;
    }
  }

  /**
   * \brief get the number of outputs from the layer
   *
   * \return the number of outputs
   *
   * this function is called by the implementations of INetworkDefinition and
   * IBuilder. In particular, it is called prior to any call to initialize().
   */
  virtual int initialize() noexcept override { return 0; }
  virtual void terminate() noexcept override {}
  int getNbOutputs() const noexcept override { return 1; }
  nvinfer1::Dims getOutputDimensions(int index,
                                     const nvinfer1::Dims *inputs,
                                     int nbInputDims) noexcept override {
    input_dims_ = inputs[0];
    for (int i = 1; i < input_dims_.nbDims; i++) {
      output_dims_.d[i] = input_dims_.d[i];
    }
    return output_dims_;
  }

  void configureWithFormat(const nvinfer1::Dims *inputDims,
                           int nbInputs,
                           const nvinfer1::Dims *outputDims,
                           int nbOutputs,
                           nvinfer1::DataType type, 
                           nvinfer1::PluginFormat format, 
                           int maxBatchSize) noexcept override {
    input_dims_ = inputDims[0];
    for (int i = 1; i < input_dims_.nbDims; i++) {
      output_dims_.d[i] = input_dims_.d[i];
    }
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

  virtual ~ArgMax1Plugin() {}

  char const* getPluginType() const noexcept override { return "Argmax"; }
  
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
  bool out_max_val_;
  size_t top_k_;
  int axis_;
  float float_min_;
  nvinfer1::Dims input_dims_;
  nvinfer1::Dims output_dims_;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
