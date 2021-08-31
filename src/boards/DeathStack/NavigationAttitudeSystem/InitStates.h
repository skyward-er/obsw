/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Marco Cella
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

/**
 * INITIALIZATION OF ATTITUDE, NOT TO USE WHILE THE ROCKET IS FLYING
 * ecompass reference: https://de.mathworks.com/help/fusion/ref/ecompass.html
 * triad reference:
 * https://www.air.iitb.ac.in/satelliteWiki/index.php/Triad_Algorithm
 */

#pragma once

#include <Common.h>
#include <configs/NASConfig.h>
#include <diagnostic/PrintLogger.h>
#include <math/SkyQuaternion.h>
#include <utils/aero/AeroUtils.h>

#include <Eigen/Dense>

using namespace Eigen;

namespace DeathStackBoard
{

using namespace NASConfigs;

class InitStates
{

public:
    InitStates();

    /**
     * @brief ecompass algorithm to estimate the attitude before the liftoff.
     *
     * @param acc 3x1 accelerometer readings [ax ay az].
     * @param mag 3x1 magnetometer readings [mx my mz].
     *
     */
    void eCompass(const Vector3f acc, const Vector3f mag);

    /**
     * @brief triad algorithm to estimate the attitude before the liftoff.
     *
     * @param acc 3x1 accelerometer readings [ax ay az].
     * @param mag 3x1 magnetometer readings [mx my mz].
     */
    void triad(const Vector3f acc, const Vector3f mag);

    /**
     * @brief Initialization of the positions before the liftoff.
     *
     * @param gps_lat Latitude.
     * @param gps_lon Longitude.
     * @param press Reference pressure [Pa].
     * @param press Reference temperature [Pa].
     */
    void positionInit(const float gps_lat, const float gps_lon,
                      const float press);  //, const float temp);
    /**
     * @brief Initialization of the velocities before the liftoff.
     *
     */
    void velocityInit();

    /**
     * @brief Initialization of the biases before the liftoff.
     *
     */
    void biasInit();

    Matrix<float, N, 1> getInitX();

private:
    Matrix<float, N, 1> x_init;
    Matrix3f R, Rb, Ri;
    SkyQuaternion quat;
    Vector4f x_quat;
    Vector3f eul;

    PrintLogger log = Logging::getLogger("deathstack.nas.initstates");
};

InitStates::InitStates() { x_init << MatrixXf::Zero(N, 1); }

void InitStates::eCompass(const Vector3f acc, const Vector3f mag)
{
    // ndr: since this method runs only when the rocket is stationary, there's
    // no need to add the gravity vector because the accelerometers already
    // measure it. This is not true if we consider the flying rocket.

    Vector3f am(acc.cross(mag));

    R << am.cross(acc), am, acc;
    R.col(0).normalize();
    R.col(1).normalize();
    R.col(2).normalize();

    x_quat = quat.rotm2quat(R);
    eul    = quat.quat2eul(x_quat);

    x_init(NL)     = x_quat(0);
    x_init(NL + 1) = x_quat(1);
    x_init(NL + 2) = x_quat(2);
    x_init(NL + 3) = x_quat(3);
}

void InitStates::triad(const Vector3f acc, const Vector3f mag)
{
    LOG_DEBUG(log, "Executing TRIAD");

    // The coulmns of the the triad matrices. b:body, i:inertial
    Vector3f t1b, t2b, t3b, t1i, t2i, t3i;

    // vettore gravità "vero" in NED, normalizzato
    // -1 su asse "down", perché da fermo l'accelerometro
    // misura la reazione vincolare (rivolta verso l'alto)
    Vector3f g_norm(0.0F, 0.0F, -1.0F);

    t1b = acc;
    t1i = g_norm;

    t2b = acc.cross(mag);
    t2b.normalize();
    t2i = g_norm.cross(NED_MAG);
    t2i.normalize();

    t3b = t2b.cross(t1b);
    t3i = t2i.cross(t1i);

    Rb << t1b, t2b, t3b;
    Ri << t1i, t2i, t3i;

    R = Rb * Ri.transpose();
    R.col(0).normalize();
    R.col(1).normalize();
    R.col(2).normalize();

    x_quat = quat.rotm2quat(R);
    eul    = quat.quat2eul(x_quat);

    x_init(NL)     = x_quat(0);
    x_init(NL + 1) = x_quat(1);
    x_init(NL + 2) = x_quat(2);
    x_init(NL + 3) = x_quat(3);
}

void InitStates::positionInit(const float gps_lat, const float gps_lon,
                              const float press)  //, const float temp)
{
    x_init(0) = EARTH_RADIUS * gps_lon * CLAT;
    x_init(1) = EARTH_RADIUS * gps_lat;
    x_init(2) = aeroutils::relAltitude(press, MSL_PRESSURE,
                                       MSL_TEMPERATURE);  // msl altitude

    // starting point centered in the launchpad
    // x_init(0) = 0.0f;
    // x_init(1) = 0.0f;
    // x_init(2) = 0.0f;  // agl altitude
}

void InitStates::velocityInit()
{
    x_init(3) = 0.0F;
    x_init(4) = 0.0F;
    x_init(5) = 0.0F;
}

void InitStates::biasInit()
{
    x_init(NL + 4) = 0.0F;
    x_init(NL + 5) = 0.0F;
    x_init(NL + 6) = 0.0F;
}

Matrix<float, N, 1> InitStates::getInitX() { return x_init; }

}  // namespace DeathStackBoard