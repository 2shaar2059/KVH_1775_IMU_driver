#ifndef _EIGEN_WRAPPER_
#define _EIGEN_WRAPPER_

#pragma GCC diagnostic push   // Eigen compares floats internally
#pragma GCC diagnostic ignored "-Wfloat-equal"
#include <eigen3/Eigen/Dense>
#pragma GCC diagnostic pop

typedef Eigen::Vector3d Vector3;
typedef Eigen::Matrix3d Matrix_3x3;

#endif   // _EIGEN_WRAPPER_