package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gflags",
    hdrs = glob([
        "include/gflags/*.h",
    ]),
    include_prefix = "",
    strip_include_prefix = "include",
    srcs = select({
        "@//:linux_arm64": glob(["lib/aarch64-linux-gnu/libgflags*.so*"]),
        "//conditions:default": glob(["lib/x86_64-linux-gnu/libgflags*.so*"]),
    }),
)
