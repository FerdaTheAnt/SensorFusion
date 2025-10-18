#include "logger/StateLogger.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using Catch::Matchers::WithinAbs;
const double EPS = 1e-6;

TEST_CASE("Testing FusedState logging") {
    StateLogger logger("test_log_file.csv");

    FusedState state;
    state.timestamp = 0.0;
    state.position = {1.0, 2.0, 3.0};
    state.orientation = Eigen::Quaterniond::Identity();
    state.velocity = {4.0, 5.0, 6.0};

    logger.log(state);

    std::ifstream file("test_log_file.csv");
    REQUIRE(file.is_open());
    std::string line;
    std::getline(file, line);
    std::getline(file, line);
    std::stringstream ss(line);
    std::string token;
    std::getline(ss, token, ',');
    REQUIRE_THAT(std::stod(token), WithinAbs(state.timestamp, EPS));
    std::getline(ss, token, ',');
    REQUIRE_THAT(std::stod(token), WithinAbs(state.position.x(), EPS));
    std::getline(ss, token, ',');
    REQUIRE_THAT(std::stod(token), WithinAbs(state.position.y(), EPS));
    std::getline(ss, token, ',');
    REQUIRE_THAT(std::stod(token), WithinAbs(state.position.z(), EPS));
}
