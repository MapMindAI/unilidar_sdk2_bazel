cc_library(
    name = "pangolin",
    srcs = select({
        "@//:linux_arm64": glob(["lib/aarch64-linux-gnu/libpango*.so*"]),
        "//conditions:default": glob(["lib/libpango*.so"]),
    }),
    hdrs = glob([
        "include/pangolin/*.h",
        "include/pangolin/**/*.h",
        "include/pangolin/**/*.hpp",
        "include/sigslot/*.hpp",
    ]),
    defines = ["HAVE_GLEW"],
    includes = [
        "include",
    ],
    linkopts = [
        "-lGL",
        "-lGLEW",
        "-Wl,-rpath,/usr/local/lib",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)
