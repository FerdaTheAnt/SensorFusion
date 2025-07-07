#pragma once
#include "sensors.hpp"
#include "complementary_filter.hpp"

inline FusedState fuseIMUandGPS(const IMUData& imu,
                                const GPSData& gps,
                                const Eigen::Quaterniond& orientation){
    FusedState fused;
    fused.timestamp = imu.timestamp;

    fused.orientation = orientation;

    fused.position = gps.position;
    fused.velocity = gps.velocity;

    return fused;
}
