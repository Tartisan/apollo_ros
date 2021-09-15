load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_based_calibrator",
    srcs = ["lane_based_calibrator.cc"],
    hdrs = ["lane_based_calibrator.h"],
    deps = [
        ":lane_struct_for_calib",
        "//kernels/perception/camera/lib/calibrator/common:histogram_estimator",
        "//kernels/perception/common/i_lib/core",
    ],
)

cc_library(
    name = "lane_struct_for_calib",
    hdrs = ["lane_struct_for_calib.h"],
    deps = [
        "@eigen",
    ],
)

cc_library(
    name = "laneline_calibrator",
    srcs = ["laneline_calibrator.cc"],
    hdrs = ["laneline_calibrator.h"],
    deps = [
        ":lane_based_calibrator",
        "//kernels/perception/camera/lib/interface",
        "//kernels/perception/common/i_lib/core",
    ],
)

cpplint()
