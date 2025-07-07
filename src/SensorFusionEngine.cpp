#include "SensorFusionEngine.hpp"
#include "complementary_filter.hpp"
#include <algorithm>

SensorFusionEngine::SensorFusionEngine()
    : current_state_{
        0.0,
        Eigen::Quaterniond::Identity(),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero()
} {}

void SensorFusionEngine::handleIMU(const IMUData& imu) {
    if(last_imu_) {
        current_state_.orientation = runComplementaryFilter(*last_imu_, imu, current_state_.orientation);
    }
    last_imu_ = imu;
    updateFusedState();
}

void SensorFusionEngine::handleGPS(const GPSData& gps) {
    last_gps_ = gps;
    updateFusedState();
}

void SensorFusionEngine::updateFusedState() {
    if (last_imu_ && last_gps_) {
        current_state_.timestamp = std::max(last_imu_->timestamp, last_gps_->timestamp);
        current_state_.position = last_gps_->position;
        current_state_.velocity = last_gps_->velocity;
    }
}

FusedState SensorFusionEngine::getCurrentState() const {
    return current_state_;
}
