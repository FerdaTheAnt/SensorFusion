#include <catch2/catch_test_macros.hpp>
#include "complementary_filter.hpp"

TEST_CASE("Complementary filter with zero gyro and gravity accel keeps identity") {
    IMUData prev{0.00, {0, 0, 9.81}, {0, 0, 0}};
    IMUData curr{0.01, {0, 0, 9.81}, {0, 0, 0}};

    auto orientation = runComplementaryFilter(prev, curr, Eigen::Quaterniond::Identity());

    CHECK(orientation.isApprox(Eigen::Quaterniond::Identity(), 1e-6));
}
