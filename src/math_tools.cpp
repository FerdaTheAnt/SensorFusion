#include "math_tools.hpp"

Eigen::Matrix<double, 3, 3> skew_symmetric_matrix(const Eigen::Vector3d& v) {
    Eigen::Matrix<double, 3, 3> v_x {
        {0, -v.z(), v.y()},
        {v.z(), 0, -v.x()},
        {-v.y(), v.x(), 0},
    };

    return v_x;
}

Eigen::Matrix<double, 4, 4> L_matrix(const Eigen::Quaterniond& p) {
    Eigen::Matrix<double, 4, 4> p_L = Eigen::Matrix<double, 4, 4>::Zero();
    p_L.block<4, 1>(0, 0) << p.w(), p.vec();
    p_L.block<1, 3>(0, 1) = -p.vec().transpose();
    p_L.block<3, 3>(1, 1) = p.w()*Eigen::Matrix3d::Identity() + skew_symmetric_matrix(p.vec());

    return p_L;
}

Eigen::Matrix<double, 4, 4> R_matrix(const Eigen::Quaterniond& q) {
    Eigen::Matrix<double, 4, 4> q_R = Eigen::Matrix<double, 4, 4>::Zero();
    q_R.block<4, 1>(0, 0) << q.w(), q.vec();
    q_R.block<1, 3>(0, 1) = -q.vec().transpose();
    q_R.block<3, 3>(1, 1) = q.w()*Eigen::Matrix3d::Identity() - skew_symmetric_matrix(q.vec());

    return q_R;
}
