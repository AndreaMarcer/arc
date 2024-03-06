/**
 * @file kalman.cpp
 * @author Andrea Marcer (marcera.andrea@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-06
 *
 * @copyright Copyright (c) 2024
 *
 */

/*****************************************************************************\
|                                   INCLUDES                                  |
\*****************************************************************************/
#include "control/kalman.hpp"

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
KF<N_x, N_z, N_u>::KF(Vector<float, N_x> x_, Matrix<float, N_x, N_x> A_,
                      Matrix<float, N_x, N_u> B_, Matrix<float, N_x, N_x> Q_,
                      Matrix<float, N_z, N_z> R_, Matrix<float, N_z, N_x> H_)
    : x{x_}, A{A_}, B{B_}, Q{Q_}, R{R_}, H{H_} {}

template <uint8_t N_x, uint8_t N_z, uint8_t N_u>
Vector<float, N_x>& KF<N_x, N_z, N_u>::predict(Vector<float, N_u> u_) {
    u = u_;

    // State Extrapolation
    x = A * x + B * u;

    // Covariance Extrapolation
    P = A * P * A.transpose() + Q;
    return x;
}

template <uint8_t N_x, uint8_t N_z, uint8_t N_u>
void KF<N_x, N_z, N_u>::update(Vector<float, N_z> z_) {
    z = z_;

    // Kalman Gain
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // Covariance Update
    P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

    // State Update
    x = x + K * (z - H * x);
}

}  // namespace arc::control