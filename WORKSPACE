workspace(name = "unilidar_sdk2")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")

# Override old gtest from transitive deps to a Bazel-6-compatible release.
http_archive(
    name = "com_google_googletest",
    sha256 = "8ad598c73ad796e0d8280b082cebd82a630d73e73cd3c70057938a6501bba5d7",
    urls = ["https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz"],
    strip_prefix = "googletest-1.14.0",
)

maybe(
    native.new_local_repository,
    name = "ros_humble",
    build_file = "//third_party:ros_humble.BUILD",
    path = "/opt/ros/humble",
)

maybe(
    native.new_local_repository,
    name = "pangolin",
    build_file = "//third_party:pangolin.BUILD",
    path = "/usr/local/",
)
