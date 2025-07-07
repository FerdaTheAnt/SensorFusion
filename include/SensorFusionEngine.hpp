#pragma once

#include "sensors.hpp"
#include <optional>

class SensorFusionEngine {
public:
    SensorFusionEngine();

    void handleIMU(const IMUData& imu);
    void handleGPS(const GPSData& gps);
    FusedState getCurrentState() const;

private:
    void updateFusedState();

    std::optional<IMUData> last_imu_;
    std::optional<GPSData> last_gps_;
    FusedState current_state_;
};
