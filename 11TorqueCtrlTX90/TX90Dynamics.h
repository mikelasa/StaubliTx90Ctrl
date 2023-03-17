#pragma once
#include "Eigen/Dense"

Eigen::MatrixXd compute_fregCentr(float  q2, float  q3, float  q4, float  q5, float q6);
Eigen::MatrixXd compute_fregCor(float  q2, float  q3, float  q4, float  q5, float q6);
Eigen::MatrixXd compute_fregFric(float  dq1, float  dq2, float  dq3, float  dq4, float  dq5, float dq6);
Eigen::MatrixXd compute_fregG(float  q2, float  q3, float  q4, float  q5, float q6);
Eigen::MatrixXd compute_Jacobian(float  dz3, float  dz4, float  dz6, float  l1, float  l2, float  q1, float  q2, float  q3, float  q4, float q5);
Eigen::MatrixXd compute_fregCentrCor(float  dq1, float  dq2, float  dq3, float  dq4, float  dq5, float  dq6, float  q2, float  q3, float  q4, float  q5, float q6);
Eigen::MatrixXd compute_fregCentrCorG(float  dq1, float  dq2, float  dq3, float  dq4, float  dq5, float  dq6, float  q2, float  q3, float  q4, float  q5, float q6);
Eigen::MatrixXd compute_fregMM(float  q2, float  q3, float  q4, float  q5, float q6);