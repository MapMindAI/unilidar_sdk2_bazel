package(default_visibility = ["//visibility:public"])

cc_import(
    name = "unilidar_sdk2_archive",
    static_library = select({
        "//toolchain:aarch64_build": "lib/aarch64/libunilidar_sdk2.a",
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


cc_library(
    name = "raw_packet_file",
    hdrs = ["raw_packet_file.h"],
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
        "@system_gflags//:gflags",
        "@system_glog//:glog",
        "@ros_humble//:ros_humble",
    ],
)


cc_binary(
    name = "unitree_lidar_packet_recorder",
    srcs = ["unitree_lidar_packet_recorder.cc"],
    deps = [
        ":raw_packet_file",
        ":unilidar_sdk2",
        "@system_gflags//:gflags",
        "@system_glog//:glog",
    ],
)

cc_binary(
    name = "unitree_lidar_packet_replayer",
    srcs = ["unitree_lidar_packet_replayer.cc"],
    deps = [
        ":raw_packet_file",
        ":unilidar_sdk2",
        "//unitree_lidar_sdk/calibration:replayer_common",
        "//unitree_lidar_sdk/calibration:replayer_viewer",
        "@system_gflags//:gflags",
        "@system_glog//:glog",
        "@eigen//:eigen",
        "@pangolin//:pangolin",
    ],
)

cc_binary(
    name = "unitree_lidar_packet_auto_calibrator",
    srcs = ["unitree_lidar_packet_auto_calibrator.cc"],
    deps = [
        "//unitree_lidar_sdk/calibration:calibration_optimizer",
        "//unitree_lidar_sdk/calibration:plane_extractor",
        "//unitree_lidar_sdk/calibration:replayer_common",
        "//unitree_lidar_sdk/calibration:replayer_viewer",
        "@system_gflags//:gflags",
        "@system_glog//:glog",
    ],
)

cc_binary(
    name = "unitree_lidar_packet_manual_calibrator",
    srcs = ["unitree_lidar_packet_manual_calibrator.cc"],
    deps = [
        "//unitree_lidar_sdk/calibration:calibration_optimizer",
        "//unitree_lidar_sdk/calibration:plane_extractor",
        "//unitree_lidar_sdk/calibration:replayer_common",
        "//unitree_lidar_sdk/calibration:replayer_viewer",
        "@system_gflags//:gflags",
        "@system_glog//:glog",
    ],
)
