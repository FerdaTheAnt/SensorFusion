#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

struct IMUData {
    double timestamp;
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;
};

struct MagnetData {
    double timestamp;
    Eigen::Vector3d field;
};

struct GPSData {
    double timestamp;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
};

struct FusedState {
    double timestamp;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
};
