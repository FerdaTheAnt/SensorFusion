#include "SensorFusionEngine.hpp"
#include "complementary_filter.hpp"
#include "sensors.hpp"
#include "math_tools.hpp"
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <mutex>
#include <iostream>

SensorFusionEngine::SensorFusionEngine()
    : method_(EngineMethod::kalman), current_state_{
        0.0,
        Eigen::Quaterniond::Identity(),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero()
}
{}

void SensorFusionEngine::setEngineMethod(const std::string& method)
{
    if(method == "kalman")
    {
        method_ = EngineMethod::kalman;
    }
    else if(method == "complementary")
    {
        method_ = EngineMethod::complementary;
    }
}


void SensorFusionEngine::computeInitialOrientation()
{
    Eigen::Vector3d g_n {0, 0, 1};
    Eigen::Vector3d m_n {1, 0, 0};
    Eigen::Vector3d g_b = last_imu_->accel.normalized();
    Eigen::Vector3d m_b = g_b.cross(last_magnet_->field.normalized().cross(g_b));

    Eigen::Quaterniond g_n_bar;
    g_n_bar.w() = 0;
    g_n_bar.vec() = g_n;

    Eigen::Quaterniond g_b_bar;
    g_b_bar.w() = 0;
    g_b_bar.vec() = g_b;

    Eigen::Quaterniond m_n_bar;
    m_n_bar.w() = 0;
    m_n_bar.vec() = m_n;

    Eigen::Quaterniond m_b_bar;
    m_b_bar.w() = 0;
    m_b_bar.vec() = m_b;

    Eigen::Matrix<double, 4, 4> A = -L_matrix(g_n_bar)*R_matrix(g_b_bar) - L_matrix(m_n_bar)*R_matrix(m_b_bar);
    A = 0.5 * (A + A.transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 4, 4>> solver(A);
    if (solver.info() != Eigen::Success) abort();

    double max_eigenvalue = solver.eigenvalues()(A.cols() - 1);
    Eigen::Vector4d max_eigenvector = solver.eigenvectors().col(A.cols() - 1);

    std::cout << "Largest Eigenvalue: " << max_eigenvalue << "\n";
    std::cout << "Corresponding Eigenvector: " << max_eigenvector << "\n";
    
    std::lock_guard<std::mutex> lock(mutex_);
    current_state_.orientation.w() = max_eigenvector(0);
    current_state_.orientation.vec() = max_eigenvector.tail<3>();
    if(method_ == EngineMethod::kalman)
    {
        kf.computeInitCovariance(current_state_.orientation);
    }
    std::cout << "Initial orientation: " << current_state_.orientation << std::endl;
}

void SensorFusionEngine::handleOrientation(const IMUData& imu,
                       const MagnetData& magnet)
{
    if(method_ == EngineMethod::complementary)
    {
        handleIMU(imu);
        last_magnet_ = magnet;
    }
    else
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if(last_imu_ && last_magnet_) {
            current_state_.orientation = kf.update(*last_imu_, imu, *last_magnet_, magnet, current_state_.orientation);
        }
        last_magnet_ = magnet;
        last_imu_ = imu;
        updateFusedState();
    }
}

void SensorFusionEngine::initHandleIMU(const IMUData& imu)
{
    last_imu_ = imu;
    current_state_.timestamp = imu.timestamp;
}

void SensorFusionEngine::initHandleMagnet(const MagnetData& magnet)
{
    last_magnet_ = magnet;
    current_state_.timestamp = magnet.timestamp;
}

void SensorFusionEngine::handleIMU(const IMUData& imu) {
    std::lock_guard<std::mutex> lock(mutex_);
    //** version for complementary filter since it does not utilize magnetometer data
    if(last_imu_) {
        current_state_.orientation = runComplementaryFilter(*last_imu_, imu, current_state_.orientation);
    }
    last_imu_ = imu;
    updateFusedState();
}

void SensorFusionEngine::handleMagnet(const MagnetData& magnet) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(last_imu_ && last_magnet_) {
        current_state_.orientation = kf.update(*last_imu_, *last_imu_, *last_magnet_, magnet, current_state_.orientation);
    }
    last_magnet_ = magnet;
    updateFusedState();
}

void SensorFusionEngine::handleGPS(const GPSData& gps) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_gps_ = gps;
    updateFusedState();
}

void SensorFusionEngine::updateFusedState() {
    if (last_imu_ && last_gps_ && last_magnet_) {
    // if (last_imu_ && last_gps_) {
        current_state_.timestamp = std::max(std::max(last_imu_->timestamp,
                                            last_magnet_->timestamp),
                                            last_gps_->timestamp);
        // current_state_.timestamp = std::max(last_imu_->timestamp,
        //                                     last_gps_->timestamp);
        current_state_.position = last_gps_->position;
        current_state_.velocity = last_gps_->velocity;
    }
}

FusedState SensorFusionEngine::getCurrentState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_state_;
}
