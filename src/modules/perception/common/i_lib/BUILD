load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "i_lib",
    deps = [
        "//kernels/perception/common/i_lib/algorithm:i_sort",
        "//kernels/perception/common/i_lib/core",
        "//kernels/perception/common/i_lib/da:i_ransac",
        "//kernels/perception/common/i_lib/geometry:i_plane",
        "//kernels/perception/common/i_lib/pc:i_ground",
    ],
)

cpplint()
