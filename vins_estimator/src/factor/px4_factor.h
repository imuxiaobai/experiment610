/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

// #include "../utility/utility.h"
// #include "../estimator/parameters.h"
// #include "integration_base.h"

// #include "../utility/geometry_math_type.h"
#include <ceres/ceres.h>
#include "../utility/geometry_math_type.h"

// class Math{
// public:
    // void _get_dcm_from_q(Eigen::Matrix3d &dcm, const Eigen::Quaterniond &q) {
    //     float a = q.w();
    //     float b = q.x();
    //     float c = q.y();
    //     float d = q.z();
    //     float aSq = a*a;
    //     float bSq = b*b;
    //     float cSq = c*c;
    //     float dSq = d*d;
    //     dcm(0, 0) = aSq + bSq - cSq - dSq; 
    //     dcm(0, 1) = 2 * (b * c - a * d);
    //     dcm(0, 2) = 2 * (a * c + b * d);
    //     dcm(1, 0) = 2 * (b * c + a * d);
    //     dcm(1, 1) = aSq - bSq + cSq - dSq;
    //     dcm(1, 2) = 2 * (c * d - a * b);
    //     dcm(2, 0) = 2 * (b * d - a * c);
    //     dcm(2, 1) = 2 * (a * b + c * d);
    //     dcm(2, 2) = aSq - bSq - cSq + dSq;
    // }

    // void _get_euler_from_R(Eigen::Vector3d &e, const Eigen::Matrix3d &R) {
    //     float phi = atan2f(R(2, 1), R(2, 2));
    //     float theta = asinf(-R(2, 0));
    //     float psi = atan2f(R(1, 0), R(0, 0));
    //     float pi = M_PI;

    //     if (fabsf(theta - pi/2.0f) < 1.0e-3) {
    //         phi = 0.0f;
    //         psi = atan2f(R(1, 2), R(0, 2));
    //     } else if (fabsf(theta + pi/2.0f) < 1.0e-3) {
    //         phi = 0.0f;
    //         psi = atan2f(-R(1, 2), -R(0, 2));
    //     }
    //     e(0) = phi;//滚转
    //     e(1) = theta;//抚养
    //     e(2) = psi;//片行
    // }

    // void _get_euler_from_q(Eigen::Vector3d &e, const Eigen::Quaterniond &q) {
    //     Eigen::Matrix3d temp_R;
    //     _get_dcm_from_q(temp_R, q);
    //     _get_euler_from_R(e, temp_R);
    // }

    // void _get_q_from_euler(Eigen::Quaterniond &q, const Eigen::Vector3d &e) {

    //     float cosPhi_2 = float(cos(e(0) / float(2.0)));
    //     float cosTheta_2 = float(cos(e(1) / float(2.0)));
    //     float cosPsi_2 = float(cos(e(2) / float(2.0)));
    //     float sinPhi_2 = float(sin(e(0) / float(2.0)));
    //     float sinTheta_2 = float(sin(e(1) / float(2.0)));
    //     float sinPsi_2 = float(sin(e(2) / float(2.0)));
    //     q.w() = cosPhi_2 * cosTheta_2 * cosPsi_2 +
    //             sinPhi_2 * sinTheta_2 * sinPsi_2;
    //     q.x() = sinPhi_2 * cosTheta_2 * cosPsi_2 -
    //             cosPhi_2 * sinTheta_2 * sinPsi_2;
    //     q.y() = cosPhi_2 * sinTheta_2 * cosPsi_2 +
    //             sinPhi_2 * cosTheta_2 * sinPsi_2;
    //     q.z() = cosPhi_2 * cosTheta_2 * sinPsi_2 -
    //             sinPhi_2 * sinTheta_2 * cosPsi_2;
    //     q.normalize();

    // }
//};

class PX4Factor : public ceres::SizedCostFunction<3, 7>
{
  public:
    PX4Factor() = delete;
    PX4Factor(Eigen::Quaterniond _px4_q):px4_q(_px4_q)
    {
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {

        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

            // std::cout << "test3" << std::endl;
        Eigen::Vector3d euler_px4;
        Eigen::Vector3d euler_raw;
        Eigen::Vector3d euler_ret;
        Eigen::Vector3d delta_euler;
        Eigen::Quaterniond tmp_q;
        // math_class->get_euler_from_q(euler_px4, px4_q);
        get_euler_from_q(euler_px4, px4_q);
        // math_class->get_euler_from_q(euler_raw, Qi);
        get_euler_from_q(euler_raw, Qi);
        euler_ret(0) = euler_px4(0);
        euler_ret(1) = euler_px4(1);
        euler_ret(2) = euler_raw(2);
        // math_class->get_q_from_euler(tmp_q, euler_ret);
        get_q_from_euler(tmp_q, euler_ret);
        Eigen::Quaterniond delta_q = tmp_q.inverse() * Qi;
        delta_q.normalize();
        // math_class->get_euler_from_q(delta_euler, delta_q);
        // std::cout << "dalta_euler" << delta_euler << std::endl;
        get_euler_from_q(delta_euler, delta_q);

        Eigen::Map<Eigen::Vector3d> residual(residuals);
        residual = delta_euler;

        Eigen::Matrix<double, 3, 3> sqrt_info;
        sqrt_info.setIdentity();
        sqrt_info = sqrt_info * 1000;
        //sqrt_info.setIdentity();
        residual = sqrt_info * residual;

        if (jacobians)
        {

            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
                jacobian_pose_i.setZero();

                jacobian_pose_i.block<3, 3>(0, 3) = Utility::Qleft(tmp_q.inverse() * Qi).bottomRightCorner<3, 3>();
                jacobian_pose_i = sqrt_info * jacobian_pose_i;
            }

        }
        

        return true;
    }
    
    Eigen::Quaterniond px4_q;
// private:
//     Math* math_class = new Math();
    // Math* math_class = new Math();
    // void get_dcm_from_q(Eigen::Matrix3d &dcm, const Eigen::Quaterniond &q) {
    //     float a = q.w();
    //     float b = q.x();
    //     float c = q.y();
    //     float d = q.z();
    //     float aSq = a*a;
    //     float bSq = b*b;
    //     float cSq = c*c;
    //     float dSq = d*d;
    //     dcm(0, 0) = aSq + bSq - cSq - dSq; 
    //     dcm(0, 1) = 2 * (b * c - a * d);
    //     dcm(0, 2) = 2 * (a * c + b * d);
    //     dcm(1, 0) = 2 * (b * c + a * d);
    //     dcm(1, 1) = aSq - bSq + cSq - dSq;
    //     dcm(1, 2) = 2 * (c * d - a * b);
    //     dcm(2, 0) = 2 * (b * d - a * c);
    //     dcm(2, 1) = 2 * (a * b + c * d);
    //     dcm(2, 2) = aSq - bSq - cSq + dSq;
    // }

    // void get_euler_from_R(Eigen::Vector3d &e, const Eigen::Matrix3d &R) {
    //     float phi = atan2f(R(2, 1), R(2, 2));
    //     float theta = asinf(-R(2, 0));
    //     float psi = atan2f(R(1, 0), R(0, 0));
    //     float pi = M_PI;

    //     if (fabsf(theta - pi/2.0f) < 1.0e-3) {
    //         phi = 0.0f;
    //         psi = atan2f(R(1, 2), R(0, 2));
    //     } else if (fabsf(theta + pi/2.0f) < 1.0e-3) {
    //         phi = 0.0f;
    //         psi = atan2f(-R(1, 2), -R(0, 2));
    //     }
    //     e(0) = phi;//滚转
    //     e(1) = theta;//抚养
    //     e(2) = psi;//片行
    // }

    // void get_euler_from_q(Eigen::Vector3d &e, const Eigen::Quaterniond &q) {
    //     Eigen::Matrix3d temp_R;
    //     get_dcm_from_q(temp_R, q);
    //     get_euler_from_R(e, temp_R);
    // }

    // void get_q_from_euler(Eigen::Quaterniond &q, const Eigen::Vector3d &e) {

    //     float cosPhi_2 = float(cos(e(0) / float(2.0)));
    //     float cosTheta_2 = float(cos(e(1) / float(2.0)));
    //     float cosPsi_2 = float(cos(e(2) / float(2.0)));
    //     float sinPhi_2 = float(sin(e(0) / float(2.0)));
    //     float sinTheta_2 = float(sin(e(1) / float(2.0)));
    //     float sinPsi_2 = float(sin(e(2) / float(2.0)));
    //     q.w() = cosPhi_2 * cosTheta_2 * cosPsi_2 +
    //             sinPhi_2 * sinTheta_2 * sinPsi_2;
    //     q.x() = sinPhi_2 * cosTheta_2 * cosPsi_2 -
    //             cosPhi_2 * sinTheta_2 * sinPsi_2;
    //     q.y() = cosPhi_2 * sinTheta_2 * cosPsi_2 +
    //             sinPhi_2 * cosTheta_2 * sinPsi_2;
    //     q.z() = cosPhi_2 * cosTheta_2 * sinPsi_2 -
    //             sinPhi_2 * sinTheta_2 * cosPsi_2;
    //     q.normalize();

    // }
};
