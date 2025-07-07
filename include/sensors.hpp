#pragma once
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>

struct IMUData {
    double timestamp;
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;
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
