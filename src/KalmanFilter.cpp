#include "KalmanFilter.hpp"
#include "sensors.hpp"
#include "math_tools.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


KalmanFilter::KalmanFilter() {
    covariance_.setIdentity();
    covariance_ *= 20.0/180 * 3.14159265;
    Q_.setIdentity();
    Q_ *= 0.1; // process noise

    R_gps_.setIdentity();
    R_gps_ *= 5.0; //GPS noise
}

void KalmanFilter::computeInitCovariance(const Eigen::Quaterniond& orientation) {
    Eigen::Matrix3d sigma_eta = Eigen::Matrix3d::Identity();
    sigma_eta *= std::pow(20.0/180 * 3.14159265, 2);
    Eigen::Matrix<double, 4, 3> diff_exp_q = Eigen::Matrix<double, 4, 3>::Zero();
    diff_exp_q.block<3, 3>(1, 0) = Eigen::Matrix3d::Identity();

    Eigen::Quaterniond orientation_c;
    orientation_c.w() = orientation.w();
    orientation_c.vec() = -orientation.vec();

    this->covariance_ = 1.0/4 * R_matrix(orientation)
                              * diff_exp_q
                              * sigma_eta
                              * diff_exp_q.transpose()
                              * R_matrix(orientation_c);
}

Eigen::Quaterniond KalmanFilter::update(const IMUData& prev,
                                        const IMUData& curr,
                                        const MagnetData& magnet_prev,
                                        const MagnetData& magnet_curr,
                                        const Eigen::Quaterniond& prev_orientation) {
    Eigen::Quaterniond orientation = prediction_step(prev, curr, prev_orientation);
    orientation = measurement_step(prev, curr, magnet_prev, magnet_curr, orientation);
    orientation = renormalize(orientation);

    return orientation;
}

Eigen::Quaterniond KalmanFilter::prediction_step(const IMUData& prev,
                                                 const IMUData& curr,
                                                 const Eigen::Quaterniond& prev_orientation) {
    const double EPS = 1e-6;
    double dt = curr.timestamp - prev.timestamp;
    Eigen::Vector3d omega = prev.gyro;

    double omega_norm = omega.norm();
    Eigen::Quaterniond dq;
    if (omega_norm > EPS) {
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(omega_norm * 0.5 * dt, omega.normalized()));
    } else {
        dq.setIdentity();
    }
    Eigen::Quaterniond gyro_orientation = (prev_orientation * dq);

    Eigen::Matrix<double, 4, 4> F = Eigen::Matrix<double, 4, 4>::Zero();
    F.block<4, 1>(0, 0) << dq.w(), dq.vec();
    F.block<1, 3>(0, 1) = -dq.vec().transpose();
    F.block<3, 3>(1, 1) = dq.w()*Eigen::Matrix3d::Identity() - skew_symmetric_matrix(dq.vec());

    Eigen::Matrix<double, 4, 4> q_L = Eigen::Matrix<double, 4, 4>::Zero();
    q_L.block<4, 1>(0, 0) << prev_orientation.w(), prev_orientation.vec();
    q_L.block<1, 3>(0, 1) = -prev_orientation.vec().transpose();
    q_L.block<3, 3>(1, 1) = prev_orientation.w()*Eigen::Matrix3d::Identity() + skew_symmetric_matrix(prev_orientation.vec());

    Eigen::Matrix<double, 4, 3> exp_q_diff_approx = Eigen::Matrix<double, 4, 3>::Zero();
    exp_q_diff_approx.block<3, 3>(1, 0) = Eigen::Matrix<double, 3, 3>::Identity();

    Eigen::Matrix<double, 4, 3> G = -0.5 * dt * q_L * exp_q_diff_approx;

    this->covariance_ = F * this->covariance_ * F.transpose() + G * this->Q_ * G.transpose(); 

    return gyro_orientation;
}

Eigen::Matrix<double, 3, 4> jacobian_rotation_quaternion(const Eigen::Quaterniond q_pred, Eigen::Vector3d v) {
    const Eigen::Vector3d& q_v = q_pred.vec();
    const double q0 = q_pred.w();

    Eigen::Matrix<double, 3, 4> H_v = Eigen::Matrix<double, 3, 4>::Zero();

    H_v.block<3, 1>(0, 0) = q0 * v + v.cross(q_v);

    H_v.block<3, 3>(0, 1) = q_v * v.transpose()
                           + q_v.dot(v) * Eigen::Matrix3d::Identity()
                           - v * q_v.transpose()
                           + q0 * skew_symmetric_matrix(v);

    return 2 * H_v;
}

Eigen::Quaterniond KalmanFilter::measurement_step(const IMUData& prev,
                                                  const IMUData& curr,
                                                  const MagnetData& mag_prev,
                                                  const MagnetData& mag_curr,
                                                  const Eigen::Quaterniond& orientation_prediction) {
    Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 4> H = Eigen::Matrix<double, 6, 4>::Zero(); 

    Eigen::Vector3d accel_est {0, 0, 1};
    H.block<3, 4>(0, 0) = jacobian_rotation_quaternion(orientation_prediction, accel_est);
    accel_est = orientation_prediction.toRotationMatrix().transpose() * accel_est;
    delta.block<3, 1>(0, 0) = curr.accel.normalized() - accel_est.normalized();

    Eigen::Vector3d mag_projection {1, 0, 0};
    Eigen::Vector3d g_hat = curr.accel.normalized();
    Eigen::Matrix3d P = Eigen::Matrix3d::Identity() - g_hat * g_hat.transpose();
    H.block<3, 4>(3, 0) = P * jacobian_rotation_quaternion(orientation_prediction, mag_projection);
    Eigen::Vector3d mag_est = P * (orientation_prediction.toRotationMatrix().transpose() * mag_projection);
    Eigen::Vector3d mag_meas = P * mag_curr.field.normalized();
    delta.block<3, 1>(3, 0) =  mag_meas.normalized()  - mag_est.normalized();

    Eigen::Matrix<double, 6, 6> sensor_noise = Eigen::Matrix<double, 6, 6>::Identity();
    sensor_noise.block<3, 3>(0, 0) = 0.01 * Eigen::Matrix<double, 3, 3>::Identity();
    sensor_noise.block<3, 3>(3, 3) = 0.05 * Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 6, 6> S = H * this->covariance_ * H.transpose() + sensor_noise;

    Eigen::Matrix<double, 4, 6> K = this->covariance_ * H.transpose() * S.inverse();

    Eigen::Vector4d kalman_gain = (K * delta);
    Eigen::Quaterniond orientation = orientation_prediction;
    orientation.w() += kalman_gain(0);
    orientation.x() += kalman_gain(1);
    orientation.y() += kalman_gain(2);
    orientation.z() += kalman_gain(3);

    this->covariance_ -= K * S * K.transpose();
    return orientation;
}

Eigen::Quaterniond KalmanFilter::renormalize(const Eigen::Quaterniond& orientation) {
    Eigen::Matrix<double, 4, 4> J = orientation.coeffsScalarFirst() * orientation.coeffsScalarFirst().transpose();
    J *= -1.0 / pow(orientation.norm(), 3);
    J += 1.0/orientation.norm() * Eigen::Matrix<double, 4, 4>::Identity();

    this->covariance_ = J * this->covariance_ * J.transpose();
    return orientation.normalized();
}
