package(default_visibility = ["//visibility:public"])

cc_library(
    name = "glog",
    hdrs = glob([
        "include/glog/*.h",
    ]),
    include_prefix = "",
    strip_include_prefix = "include",
    srcs = select({
        "@//:linux_arm64": glob(["lib/aarch64-linux-gnu/libglog*.so*"]),
        "//conditions:default": glob(["lib/x86_64-linux-gnu/libglog*.so*"]),
    }),
)
