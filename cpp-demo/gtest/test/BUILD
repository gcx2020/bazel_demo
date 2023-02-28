cc_test(
    name = "unit_test",
    srcs = [
        "test_greet.cc",
        "test_mock.cc",
    ],
    copts = [
        "-std=c++11",
    ],
    deps = [
        "//lib:greet",
        "@gtest",
        "@gtest//:gtest_main",
    ],
)