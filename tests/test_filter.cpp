#include "complementary_filter.hpp"
#include <catch2/catch_test_macros.hpp>

const double EPS = 1e-6;
const double PI = 3.14159265;

TEST_CASE("Complementary filter with zero gyro and gravity accel keeps identity") {
    IMUData prev{0.00, {0, 0, 9.81}, {0, 0, 0}};
    IMUData curr{0.01, {0, 0, 9.81}, {0, 0, 0}};

    auto orientation = runComplementaryFilter(prev, curr, Eigen::Quaterniond::Identity());

    CHECK(orientation.isApprox(Eigen::Quaterniond::Identity(), EPS));
}

TEST_CASE("Complementary filter with rotation around one axis") {
    double angular_speed = PI/2;
    IMUData prev{0.00, {0, 0, 9.81}, {0, 0, 0}};
    IMUData curr{0.01, {0, 0, 9.81}, {0, 0, angular_speed}};

    auto orientation = runComplementaryFilter(prev, curr, Eigen::Quaterniond::Identity());

    double dt = curr.timestamp - prev.timestamp;
    double angle = dt * angular_speed;
    
    Eigen::AngleAxisd rotation(angle, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond gyro_quaternion(rotation);
    Eigen::Quaterniond result_quaternion = gyro_quaternion.slerp(1.0-0.98, Eigen::Quaterniond::Identity());

    CHECK(orientation.isApprox(result_quaternion, EPS));
}
