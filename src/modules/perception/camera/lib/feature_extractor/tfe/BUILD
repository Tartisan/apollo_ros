load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "tracking_feature_proto",
    deps = [
        ":tracking_feature_proto_lib",
    ],
)

proto_library(
    name = "tracking_feature_proto_lib",
    srcs = ["tracking_feature.proto"],
)

cc_library(
    name = "external_feature_extractor",
    srcs = ["external_feature_extractor.cc"],
    hdrs = ["external_feature_extractor.h"],
    deps = [
        ":tracking_feature_proto",
        "//cyber/common:file",
        "//kernels/perception/camera/common",
        "//kernels/perception/camera/lib/interface",
        "//kernels/perception/inference:inference_factory_lib",
        "//kernels/perception/inference:inference_lib",
        "//kernels/perception/inference/utils:inference_resize_lib",
        "//kernels/perception/inference/utils:inference_util_lib",
    ],
)

cc_library(
    name = "project_feature",
    srcs = ["project_feature.cc"],
    hdrs = ["project_feature.h"],
    deps = [
        ":tracking_feature_proto",
        "//cyber/common:file",
        "//kernels/perception/camera/common",
        "//kernels/perception/camera/lib/interface",
        "//kernels/perception/inference:inference_factory_lib",
        "//kernels/perception/inference/utils:inference_gemm_lib",
        "//kernels/perception/inference/utils:inference_util_lib",
    ],
)

cc_library(
    name = "tracking_feat_extractor",
    srcs = ["tracking_feat_extractor.cc"],
    hdrs = ["tracking_feat_extractor.h"],
    deps = [
        ":tracking_feature_proto",
        "//cyber/common:file",
        "//kernels/perception/base",
        "//kernels/perception/camera/common",
        "//kernels/perception/camera/lib/interface",
        "//kernels/perception/inference:inference_factory_lib",
        "//kernels/perception/inference/operators:perception_inference_operators",
        "//kernels/perception/inference/utils:inference_gemm_lib",
        "//kernels/perception/inference/utils:inference_util_lib",
    ],
)

cpplint()
