package(default_visibility = ["//visibility:public"])

cc_library(
    name = "eigen",
    hdrs = glob([
        "include/eigen3/Eigen/**/*.h",
        "include/eigen3/Eigen/**/*.hpp",
        "include/eigen3/unsupported/Eigen/**/*.h",
        "include/eigen3/unsupported/Eigen/**/*.hpp",
    ]) + glob([
        "include/eigen3/Eigen/*",
        "include/eigen3/unsupported/Eigen/*",
    ]),
    include_prefix = "",
    strip_include_prefix = "include/eigen3",
)
