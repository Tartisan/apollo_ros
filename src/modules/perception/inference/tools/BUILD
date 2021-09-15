load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cal_table_generator",
    srcs = ["cal_table_generator.cc"],
    linkstatic = False,
    deps = [
        "//kernels/perception/inference:inference_factory_lib",
        "//kernels/perception/inference:inference_lib",
        "//kernels/perception/inference/tensorrt:batch_stream",
        "//kernels/perception/inference/tensorrt:entropy_calibrator",
        "//kernels/perception/inference/tensorrt:rt_net",
        "//kernels/perception/inference/utils:inference_util_lib",
    ],
)

cc_binary(
    name = "denseline_sample",
    srcs = ["denseline_sample.cc"],
    linkstatic = False,
    deps = [
        "//kernels/perception/inference:inference_factory_lib",
        "//kernels/perception/inference:inference_lib",
        "//kernels/perception/inference/tensorrt:batch_stream",
        "//kernels/perception/inference/tensorrt:entropy_calibrator",
        "//kernels/perception/inference/tensorrt:rt_net",
        "//kernels/perception/inference/utils:inference_util_lib",
    ],
)

cc_binary(
    name = "lane_sample",
    srcs = ["lane_sample.cc"],
    linkstatic = False,
    deps = [
        "//kernels/perception/inference:inference_factory_lib",
        "//kernels/perception/inference:inference_lib",
        "//kernels/perception/inference/tensorrt:batch_stream",
        "//kernels/perception/inference/tensorrt:entropy_calibrator",
        "//kernels/perception/inference/tensorrt:rt_net",
        "//kernels/perception/inference/utils:inference_util_lib",
    ],
)

cc_binary(
    name = "yolo_sample",
    srcs = ["yolo_sample.cc"],
    linkstatic = False,
    deps = [
        "//kernels/perception/inference:inference_factory_lib",
        "//kernels/perception/inference:inference_lib",
        "//kernels/perception/inference/tensorrt:batch_stream",
        "//kernels/perception/inference/tensorrt:entropy_calibrator",
        "//kernels/perception/inference/tensorrt:rt_net",
        "//kernels/perception/inference/utils:inference_util_lib",
    ],
)

cpplint()
