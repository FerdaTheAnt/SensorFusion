#pragma once
#include "sensors.hpp"
#include <cmath>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Geometry/AngleAxis.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>

inline Eigen::Quaterniond runComplementaryFilter(const IMUData& prev,
                                         const IMUData& curr,
                                         const Eigen::Quaterniond& prev_orientation,
                                         double alpha = 0.98){
    double dt = curr.timestamp - prev.timestamp;
    Eigen::Vector3d gyro = curr.gyro;

    Eigen::Vector3d delta_angle = gyro * dt;
    Eigen::AngleAxisd delta_rot(delta_angle.norm(), delta_angle.normalized());
    Eigen::Quaterniond gyro_orientation = prev_orientation * Eigen::Quaterniond(delta_rot);
    gyro_orientation.normalize();

    double ax = curr.accel.x();
    double ay = curr.accel.y();
    double az = curr.accel.z();
    double pitch = atan2(-ax, sqrt(ay*ay + az*az));
    double roll = atan2(ay, az);

    Eigen::Quaterniond accel_orientation = 
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    Eigen::Quaterniond fused = gyro_orientation.slerp(1.0 - alpha, accel_orientation);

    return fused.normalized();
}
