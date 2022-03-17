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

#include "modules/perception/inference/tensorrt/rt_common.h"

namespace apollo {
namespace perception {
namespace inference {

class SoftmaxPlugin : public nvinfer1::IPluginV2 {
 public:
  SoftmaxPlugin(const SoftmaxParameter &param, nvinfer1::Dims in_dims) {
    input_dims_.nbDims = in_dims.nbDims;
    for (int i = 0; i < in_dims.nbDims; i++) {
      input_dims_.d[i] = in_dims.d[i];
      // input_dims_.type[i] = in_dims.type[i];
    }
    axis_ = param.axis() - 1;
    CHECK_GE(axis_, 0);
    CHECK_LE(axis_ + 1, input_dims_.nbDims);

    inner_num_ = 1;
    for (int i = axis_ + 1; i < input_dims_.nbDims; i++) {
      inner_num_ *= input_dims_.d[i];
    }
    outer_num_ = 1;
    for (int i = 0; i < axis_; i++) {
      outer_num_ *= input_dims_.d[i];
    }
    cudnnCreateTensorDescriptor(&input_desc_);
    cudnnCreateTensorDescriptor(&output_desc_);
  }

  SoftmaxPlugin() {}

  ~SoftmaxPlugin() {
    cudnnDestroyTensorDescriptor(input_desc_);
    cudnnDestroyTensorDescriptor(output_desc_);
  }
  virtual int initialize() noexcept override {
    cudnnCreate(&cudnn_);  // initialize cudnn and cublas
    cublasCreate(&cublas_);
    return 0;
  }
  virtual void terminate() noexcept override {
    cublasDestroy(cublas_);
    cudnnDestroy(cudnn_);
  }
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

  char const* getPluginType() const noexcept override { return "Softmax"; }
  
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
  cudnnHandle_t cudnn_;
  cublasHandle_t cublas_;
  nvinfer1::Dims input_dims_;
  int axis_;
  int inner_num_;
  int outer_num_;
  cudnnTensorDescriptor_t input_desc_;
  cudnnTensorDescriptor_t output_desc_;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
