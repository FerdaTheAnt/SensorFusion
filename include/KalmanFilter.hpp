#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "sensors.hpp"

class KalmanFilter {
public:
    KalmanFilter();

    Eigen::Quaterniond prediction_step(const IMUData& prev,
                                       const IMUData& curr,
                                       const Eigen::Quaterniond& prev_orientation); 
    Eigen::Quaterniond measurement_step(const IMUData& prev,
                                        const IMUData& curr,
                                        const MagnetData& mag_prev,
                                        const MagnetData& mag_curr,
                                        const Eigen::Quaterniond& orientation_prediction); 
    Eigen::Quaterniond update(const IMUData& prev,
                              const IMUData& curr,
                              const MagnetData& mag_prev,
                              const MagnetData& mag_curr,
                              const Eigen::Quaterniond& prev_orientation);
    Eigen::Quaterniond renormalize(const Eigen::Quaterniond& orientation);
    void computeInitCovariance(const Eigen::Quaterniond& orientation);
    void UpdateGPS(const GPSData& gps);

private:
    Eigen::Matrix<double, 4, 4> covariance_;
    Eigen::Matrix<double, 3, 3> Q_;
    Eigen::Matrix3d R_gps_;
    Eigen::Vector3d mag_ref_;
};
