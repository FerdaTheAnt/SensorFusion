#pragma once

#include "sensors.hpp"
#include "KalmanFilter.hpp"
#include <optional>
#include <mutex>

class SensorFusionEngine {
public:
    SensorFusionEngine();

    void handleOrientation(const IMUData& imu,
                           const MagnetData& magnet);
    void handleIMU(const IMUData& imu);
    void handleMagnet(const MagnetData& magnet);
    void handleGPS(const GPSData& gps);
    FusedState getCurrentState() const;
    void computeInitialOrientation();

    void initHandleIMU(const IMUData& imu);
    void initHandleMagnet(const MagnetData& magnet);

private:
    void updateFusedState();

    std::optional<IMUData> last_imu_;
    std::optional<MagnetData> last_magnet_;
    std::optional<GPSData> last_gps_;
    FusedState current_state_;

    KalmanFilter kf;

    mutable std::mutex mutex_;
};
