#ifndef GEOMETRY_MATH_TYPE_H_
#define GEOMETRY_MATH_TYPE_H_
// #pragma once
// #include <Eigen/Core>
// #include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <math.h>

void get_dcm_from_q(Eigen::Matrix3d &dcm, const Eigen::Quaterniond &q);

void get_q_from_dcm(Eigen::Quaterniond &q, const Eigen::Matrix3d &dcm);

void get_euler_from_R(Eigen::Vector3d &e, const Eigen::Matrix3d &R);

void get_euler_from_q(Eigen::Vector3d &e, const Eigen::Quaterniond &q);

void get_q_from_euler(Eigen::Quaterniond &q, const Eigen::Vector3d &e);

void get_dcm_from_euler(Eigen::Matrix3d &R, const Eigen::Vector3d &e);

#endif