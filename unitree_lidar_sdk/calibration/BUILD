package(default_visibility = ["//visibility:public"])

cc_library(
    name = "replayer_common",
    srcs = ["replayer_common.cc"],
    hdrs = ["replayer_common.h"],
    deps = [
        "//unitree_lidar_sdk:raw_packet_file",
        "//unitree_lidar_sdk:unilidar_sdk2",
        "@eigen//:eigen",
        "@system_glog//:glog",
    ],
)

cc_library(
    name = "plane_extractor",
    srcs = ["plane_extractor.cc"],
    hdrs = ["plane_extractor.h"],
    deps = [
        ":replayer_common",
        "@eigen//:eigen",
        "@system_glog//:glog",
    ],
)

cc_library(
    name = "calibration_optimizer",
    srcs = ["calibration_optimizer.cc"],
    hdrs = ["calibration_optimizer.h"],
    deps = [
        ":plane_extractor",
        ":replayer_common",
        "@system_glog//:glog",
    ],
)

cc_library(
    name = "replayer_viewer",
    srcs = ["replayer_viewer.cc"],
    hdrs = ["replayer_viewer.h"],
    deps = [
        ":calibration_optimizer",
        ":plane_extractor",
        ":replayer_common",
        "@pangolin//:pangolin",
    ],
)
