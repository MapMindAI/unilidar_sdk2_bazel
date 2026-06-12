package(default_visibility = ["//visibility:public"])

cc_import(
    name = "unilidar_sdk2_archive",
    static_library = select({
        "@bazel_platforms//platforms:linux_arm64": "lib/aarch64/libunilidar_sdk2.a",
        "//conditions:default": "lib/x86_64/libunilidar_sdk2.a",
    }),
)

cc_library(
    name = "unilidar_sdk2",
    hdrs = glob([
        "include/*.h",
        "include/*.hpp",
    ]),
    includes = ["include"],
    deps = [
        ":unilidar_sdk2_archive",
    ],
)

cc_binary(
    name = "example_lidar_udp",
    srcs = ["examples/example_lidar_udp.cpp"],
    deps = [":unilidar_sdk2"],
)

cc_binary(
    name = "example_lidar_serial",
    srcs = ["examples/example_lidar_serial.cpp"],
    deps = [":unilidar_sdk2"],
)

cc_binary(
    name = "set_ip_address",
    srcs = ["examples/set_ip_address.cpp"],
    deps = [":unilidar_sdk2"],
)

cc_binary(
    name = "set_to_serial_mode",
    srcs = ["examples/set_to_serial_mode.cpp"],
    deps = [":unilidar_sdk2"],
)

cc_binary(
    name = "set_to_udp_mode",
    srcs = ["examples/set_to_udp_mode.cpp"],
    deps = [":unilidar_sdk2"],
)

cc_binary(
    name = "unitree_lidar_rosnode",
    srcs = ["unitree_lidar_rosnode.cc"],
    deps = [
        ":unilidar_sdk2",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_glog//:glog",
        "@ros_humble//:ros_humble",
    ],
)
