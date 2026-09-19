#include <Eigen/Dense>
#include <Eigen/Geometry>

Eigen::Matrix<double, 3, 3> skew_symmetric_matrix(const Eigen::Vector3d& v);
Eigen::Matrix<double, 4, 4> L_matrix(const Eigen::Quaterniond& v);
Eigen::Matrix<double, 4, 4> R_matrix(const Eigen::Quaterniond& q);
