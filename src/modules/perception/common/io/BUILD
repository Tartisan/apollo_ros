load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "io_util",
    srcs = ["io_util.cc"],
    hdrs = ["io_util.h"],
    deps = [
        "//cyber",
        "//kernels/perception/base:camera",
        "//kernels/perception/base:distortion_model",
        "//kernels/perception/base:omnidirectional_model",
        "//kernels/perception/common/geometry:basic",
        "@boost",
        "@com_google_absl//absl/strings",
        "@eigen",
        "@yaml_cpp//:yaml",
    ],
)

cc_test(
    name = "io_util_test",
    size = "small",
    srcs = ["io_util_test.cc"],
    data = [
        "//kernels/perception:perception_testdata",
    ],
    deps = [
        ":io_util",
        "@gtest//:main",
    ],
)

cpplint()
