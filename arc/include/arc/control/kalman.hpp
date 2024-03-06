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
#include "Eigen/Core"

/*****************************************************************************\
|                                    MACROS                                   |
\*****************************************************************************/

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
       Matrix<float, N_x, N_u> B_, Matrix<float, N_x, N_x> Q,
       Matrix<float, N_z, N_z> R, Matrix<float, N_z, N_x> H);

    /**
     * @brief State Extrapolation
     *
     * @return Vector<float, N_x>&
     */
    Vector<float, N_x> &predict(Vector<float, N_u> u);

    /**
     * @brief State Extrapolation
     */
    void update(Vector<float, N_z> z);

private:
    /**
     * @brief State Vector
     */
    Vector<float, N_x> x = Vector<float, N_x>::Zero();

    /**
     * @brief Measurements Vector
     */
    Vector<float, N_z> z = Vector<float, N_z>::Zero();

    /**
     * @brief State Transition Matrix
     */
    Matrix<float, N_x, N_x> A = Matrix<float, N_x, N_x>::Zero();

    /**
     * @brief Input Variable
     */
    Vector<float, N_u> u = Vector<float, N_u>::Zero();

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
     * @brief Process Noise Vector
     */
    Vector<float, N_x> w = Vector<float, N_x>::Zero();

    /**
     * @brief Measurement Noise Vector
     */
    Vector<float, N_z> v = Vector<float, N_z>::Zero();

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