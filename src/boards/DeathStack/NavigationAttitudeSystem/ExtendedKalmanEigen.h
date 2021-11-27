/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Alessandro Del Duca, Luca Conterio, Marco Cella
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <configs/NASConfig.h>
#include <math/SkyQuaternion.h>
#include <utils/aero/AeroUtils.h>

#include <Eigen/Dense>

using namespace Eigen;
using namespace Boardcore;

namespace DeathStackBoard
{

using namespace NASConfigs;

using VectorNf = Matrix<float, N, 1>;

class ExtendedKalmanEigen
{

public:
    ExtendedKalmanEigen();

    /**
     * @brief Prediction step of the EKF.
     *
     * @param u 3x1 Vector of the accelerometer readings [ax ay az].
     */
    void predict(const Vector3f& u);

    /**
     * @brief EKF correction of the barometer data.
     *
     * @param y Pressure read from the barometer [Pa]
     * @param msl_press Pressure at sea level [Pa]
     * @param msl_temp Temperature at sea level [Kelvin]
     */
    void correctBaro(const float y, const float msl_press,
                     const float msl_temp);

    /**
     * @brief EKF correction of the gps readings.
     *
     * @param y 4x1 Vector of the gps readings [longitude, latitude,
     * gps_nord_vel, gps_east_vel].
     * @param sats_num Number of satellites available
     */
    void correctGPS(const Vector4f& y, const uint8_t sats_num);

    /**
     * @brief Prediction step of the Multiplicative EKF.
     *
     * @param u 3x1 Vector of the gyroscope readings [wx wy wz].
     */
    void predictMEKF(const Vector3f& u);

    /**
     * @brief MEKF correction of the magnetometer readings.
     *
     * @param y 3x1 Vector of the magnetometer readings [mx my mz].
     */
    void correctMEKF(const Vector3f& y);

    /**
     * @return 13x1 State vector [px py pz vx vy vz qx qy qz qw bx by bz].
     */
    const VectorNf& getState();

    /**
     * @param x 13x1 State vector [px py pz vx vy vz qx qy qz qw bx by bz].
     */
    void setX(const VectorNf& x);

private:
    VectorNf x;
    Matrix<float, NP, NP> P;
    Matrix<float, NL, NL> F;
    Matrix<float, NL, NL> Ftr;

    Matrix3f P_pos;
    Matrix3f P_vel;
    Matrix3f P_att;
    Matrix3f P_bias;
    Matrix<float, NL, NL> Plin;

    Matrix3f Q_pos;
    Matrix3f Q_vel;
    Matrix<float, NL, NL> Q_lin;

    Vector3f g;
    Matrix2f eye2;
    Matrix3f eye3;
    Matrix4f eye4;
    Matrix<float, 6, 6> eye6;

    Matrix<float, NBAR, NBAR> R_bar;

    Matrix<float, NGPS, NGPS> R_gps;
    Matrix<float, NGPS, NL> H_gps;
    Matrix<float, NL, NGPS> H_gpstr;

    Vector4f q;
    Matrix<float, NMAG, NMAG> R_mag;
    Matrix<float, NMEKF, NMEKF> Q_mag;
    Matrix<float, NMEKF, NMEKF> Fatt;
    Matrix<float, NMEKF, NMEKF> Fatttr;
    Matrix<float, NMEKF, NMEKF> Gatt;
    Matrix<float, NMEKF, NMEKF> Gatttr;
    Matrix<float, NMEKF, NMEKF> Patt;

    SkyQuaternion quater;
};

}  // namespace DeathStackBoard