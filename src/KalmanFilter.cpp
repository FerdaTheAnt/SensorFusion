#include "KalmanFilter.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Quaternion.h>


KalmanFilter::KalmanFilter() {
    covariance_.setIdentity();
    Q_.setIdentity();
    Q_ *= 0.01; // process noise

    R_gps_.setIdentity();
    R_gps_ *= 5.0; //GPS noise
}

Eigen::Quaterniond KalmanFilter::update(const IMUData& prev,
                                        const IMUData& curr,
                                        const Eigen::Quaterniond& prev_orientation) {
    Eigen::Quaterniond orientation = prediction_step(prev, curr, prev_orientation);
    //orientation = measurement_step(prev, curr, orientation);
    orientation = renormalize(orientation);
    
    //acc_world.z() -= 9.8066;

    return orientation;
}

Eigen::Matrix<double, 3, 3> skew_symmetric_matrix(const Eigen::Vector3d v) {
    Eigen::Matrix<double, 3, 3> v_x {
        {0, -v.z(), v.y()},
        {v.z(), 0, -v.x()},
        {-v.y(), v.x(), 0},
    };

    return v_x;
}

Eigen::Quaterniond KalmanFilter::prediction_step(const IMUData& prev,
                                             const IMUData& curr,
                                             const Eigen::Quaterniond& prev_orientation) {
    const double EPS = 1e-6;
    double dt = curr.timestamp - prev.timestamp;
    Eigen::Vector3d omega = curr.gyro;

    double omega_norm = omega.norm();
    Eigen::Quaterniond dq;
    if (omega_norm > EPS) {
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(omega_norm * 0.5 * dt, omega.normalized()));
    } else {
        dq.setIdentity();
    }
    Eigen::Quaterniond gyro_orientation = (prev_orientation * dq).normalized();

    Eigen::Matrix<double, 4, 4> F {
        {1, -gyro_orientation.x(), -gyro_orientation.y(), -gyro_orientation.z()},
        {gyro_orientation.x(), 1, -gyro_orientation.z(), gyro_orientation.y()},
        {gyro_orientation.y(), gyro_orientation.z(), 1, -gyro_orientation.x()},
        {gyro_orientation.z(), -gyro_orientation.y(), gyro_orientation.x(), 1},
    };

    // TODO: edit this
    Eigen::Matrix<double, 4, 4> q_L = Eigen::Matrix<double, 4, 4>::Zero();
    q_L.block<4, 1>(0, 0) = prev_orientation.coeffs();
    q_L.block<1, 3>(0, 1) = -prev_orientation.vec().transpose();
    q_L.block<3, 3>(1, 1) = prev_orientation.w()*Eigen::Matrix3d::Identity() + skew_symmetric_matrix(prev_orientation.vec());

    Eigen::Matrix<double, 4, 3> exp_q_diff_approx = Eigen::Matrix<double, 4, 3>::Zero();
    exp_q_diff_approx.block<3, 3>(1, 0) = Eigen::Matrix<double, 3, 3>::Identity();

    Eigen::Matrix<double, 4, 3> G = -0.5 * dt * q_L * exp_q_diff_approx;

    this->covariance_ = F * this->covariance_ * F.transpose() + G * this->Q_ * G.transpose(); 

    return gyro_orientation;
}

Eigen::Matrix<double, 3, 4> jacobian_rotation_quaternion(const Eigen::Quaterniond q_pred, Eigen::Vector3d v) {
    Eigen::Vector3d u = v.cross(q_pred.vec());
    
    Eigen::Matrix<double, 3, 4> H_v = Eigen::Matrix<double, 3, 4>::Zero();
    H_v.block<3, 1>(0, 0) = u; 
    H_v.block<3, 3>(0, 1) = (skew_symmetric_matrix(u + q_pred.w()*v) +
                             (q_pred.vec().dot(v))*Eigen::Matrix3d::Identity() - 
                             v*q_pred.vec().transpose());
    return 2*H_v;
}

Eigen::Quaterniond KalmanFilter::measurement_step(const IMUData& prev,
                                                    const IMUData& curr,
                                                    const Eigen::Quaterniond& orientation_prediction) {
    Eigen::Matrix<double, 3, 4> H = jacobian_rotation_quaternion(orientation_prediction, curr.accel);

    Eigen::Matrix<double, 3, 3> S = H * this->covariance_ * H.transpose(); // + Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 4, 3> K = this->covariance_ * H.transpose() * S.inverse();

    Eigen::Quaterniond kalman_gain = Eigen::Quaterniond(K * (curr.gyro - prev.gyro));
    Eigen::Quaterniond orientation = orientation_prediction;
    orientation.w() += kalman_gain.w();
    orientation.x() += kalman_gain.x();
    orientation.y() += kalman_gain.y();
    orientation.z() += kalman_gain.z();

    return orientation;
}

Eigen::Quaterniond KalmanFilter::renormalize(const Eigen::Quaterniond& orientation) {
    Eigen::Matrix<double, 4, 4> J = orientation.coeffs() * orientation.coeffs().transpose();
    J *= 1.0 / pow(orientation.norm(), 3);

    this->covariance_ = J * this->covariance_ * J.transpose();
    return orientation.normalized();
}
