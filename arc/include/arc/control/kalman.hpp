/**
 * @file kalman.hpp
 * @author Andrea Marcer (marcera.andrea@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/*****************************************************************************\
|                                   INCLUDES                                  |
\*****************************************************************************/
#include "common/log.hpp"
#include "Eigen/Eigen"

namespace arc::control {

/*****************************************************************************\
|                                Kalman Filter                                |
\*****************************************************************************/

using Eigen::Matrix;
using Eigen::Vector;

template <uint8_t N_x, uint8_t N_z, uint8_t N_u>
class KF {
public:
    KF(Vector<float, N_x> x_, Matrix<float, N_x, N_x> A_,
       Matrix<float, N_x, N_u> B_, Matrix<float, N_x, N_x> P_,
       Matrix<float, N_x, N_x> Q_, Matrix<float, N_z, N_z> R_,
       Matrix<float, N_z, N_x> H_)
        : x{x_}, A{A_}, B{B_}, P{P_}, Q{Q_}, R{R_}, H{H_} {};

    /**
     * @brief State Extrapolation
     *
     * @return Vector<float, N_x>&
     */
    Vector<float, N_x> &predict(Vector<float, N_u> u) {
        // State Extrapolation
        x = A * x + B * u;

        // Covariance Extrapolation
        P = A * P * A.transpose() + Q;
        return x;
    }

    /**
     * @brief State Extrapolation
     *
     * @return Vector<float, N_x>&
     */
    Vector<float, N_x> &predict() {
        // State Extrapolation
        x = A * x;

        // Covariance Extrapolation
        P = A * P * A.transpose() + Q;
        return x;
    }

    /**
     * @brief State Extrapolation
     */
    void update(Vector<float, N_z> z) {
        // Kalman Gain
        Matrix<float, N_x, N_z> m1 = P * H.transpose();
        K = m1 * (H * m1 + R).inverse();

        // Covariance Update
        Matrix<float, N_x, N_x> m2 = I - K * H;
        Matrix<float, N_x, N_x> m3 = m2 * P * m2.transpose();
        P = m3 + K * R * K.transpose();

        // State Update
        x = x + K * (z - H * x);
    }

    void printState() {
        log_info << x.transpose() << "\n";
        log_info << std::endl;
    }

    void printEstimateCovariance() {
        for (uint8_t r = 0; r < P.rows(); r++) log_info << P.row(r) << "\n";
        log_info << std::endl;
    }

    void printKalmanGain() {
        for (uint8_t r = 0; r < K.rows(); r++) log_info << K.row(r) << "\n";
        log_info << std::endl;
    }

private:
    /**
     * @brief State Vector
     */
    Vector<float, N_x> x = Vector<float, N_x>::Zero();

    /**
     * @brief Measurements Vector
     */
    // Vector<float, N_z> z = Vector<float, N_z>::Zero();

    /**
     * @brief State Transition Matrix
     */
    Matrix<float, N_x, N_x> A = Matrix<float, N_x, N_x>::Zero();

    /**
     * @brief Input Variable
     */
    // Vector<float, N_u> u = Vector<float, N_u>::Zero();

    /**
     * @brief Control Matrix
     */
    Matrix<float, N_x, N_u> B = Matrix<float, N_x, N_u>::Zero();

    /**
     * @brief Estimate Covariance
     */
    Matrix<float, N_x, N_x> P = Matrix<float, N_x, N_x>::Zero();

    /**
     * @brief Process Noise Covariance
     */
    Matrix<float, N_x, N_x> Q = Matrix<float, N_x, N_x>::Zero();

    /**
     * @brief Measurement Covariance
     */
    Matrix<float, N_z, N_z> R = Matrix<float, N_z, N_z>::Zero();

    /**
     * @brief Observation Matrix
     */
    Matrix<float, N_z, N_x> H = Matrix<float, N_z, N_x>::Zero();

    /**
     * @brief Kalman Gain
     */
    Matrix<float, N_x, N_z> K = Matrix<float, N_x, N_z>::Zero();

    /**
     * @brief Discrete-Time Index
     */
    uint32_t t{0};

    Matrix<float, N_x, N_x> I = Matrix<float, N_x, N_x>::Identity();
};

}  // namespace arc::control