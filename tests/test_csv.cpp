#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <limits>
#include "csv_loader.hpp"

using Catch::Matchers::WithinAbs;
const double EPS = std::numeric_limits<float>::epsilon()*100;

TEST_CASE("CSV IMU Loader parses empty file") {
    auto data = loadIMUCSV("tests/data/empty.csv");
    REQUIRE(data.empty());
}

TEST_CASE("CSV IMU Loader parses single line file") {
    auto data = loadIMUCSV("tests/data/line_imu.csv");
    REQUIRE(!data.empty());
    CHECK_THAT(data[0].timestamp, WithinAbs(0, EPS));

    CHECK_THAT(data[0].accel.x(), WithinAbs(1.0, EPS));
    CHECK_THAT(data[0].accel.y(), WithinAbs(1.0, EPS));
    CHECK_THAT(data[0].accel.z(), WithinAbs(9.81, EPS));

    CHECK_THAT(data[0].gyro.x(), WithinAbs(1.0, EPS));
    CHECK_THAT(data[0].gyro.y(), WithinAbs(1.0, EPS));
    CHECK_THAT(data[0].gyro.z(), WithinAbs(1.0, EPS));
}

TEST_CASE("CSV GPS Loader parses empty file") {
    auto data = loadGPSCSV("tests/data/empty.csv");
    REQUIRE(data.empty());
}

TEST_CASE("CSV GPS Loader parses single line file") {
    auto data = loadGPSCSV("tests/data/line_gps.csv");
    REQUIRE(!data.empty());
    CHECK_THAT(data[0].timestamp, WithinAbs(0, EPS));

    CHECK_THAT(data[0].position.x(), WithinAbs(0, EPS));
    CHECK_THAT(data[0].position.y(), WithinAbs(0, EPS));
    CHECK_THAT(data[0].position.z(), WithinAbs(0, EPS));

    CHECK_THAT(data[0].velocity.x(), WithinAbs(0, EPS));
    CHECK_THAT(data[0].velocity.y(), WithinAbs(0, EPS));
    CHECK_THAT(data[0].velocity.z(), WithinAbs(0, EPS));
}
