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

#pragma once

#include <utils/Constants.h>

#include "Eigen/Dense"

namespace DeathStackBoard
{

namespace NASConfigs
{

static const unsigned int NAS_UPDATE_PERIOD = 20;  // ms -> 50 hz

static const float SAMPLE_RATE = 1000.0F / NAS_UPDATE_PERIOD;  // [Hz]

static const float T = 1.0F / SAMPLE_RATE;  // [s]

static const uint32_t CALIBRATION_N_SAMPLES = 500;

static const float SIGMA_BETA = 0.0001;  // 0.01;  // [Rad/s^2]
                                         // Estimated gyroscope bias variance

static const float SIGMA_MAG = 0.7089F * 10;  // [uT^2]
                                              // Estimated magnetometer variance

static const float SIGMA_W = 0.3;
// 0.000027927F * 100;  // [rad^2]
// Estimated gyroscope variance

static const float SIGMA_GPS = 10.0F;  // [m^2]
                                       // Estimated GPS variance

static const float SIGMA_BAR = 4.3F;  // [m^2]
                                      // Estimated altitude variance

static const float SIGMA_POS = 10.0;
// 0.0192F * 100;  // [m^2]
// Estimated variance of the position noise

static const float SIGMA_VEL = 10.0;
// 0.096F * 100;  // [(m/s)^2]
// Estimated variance of the velocity noise

static const float P_POS          = 1.0F;  // Position prediction covariance
static const float P_POS_VERTICAL = 10.0F;

static const float P_VEL          = 1.0F;  // Velocity prediction covariance
static const float P_VEL_VERTICAL = 10.0F;

static const float P_ATT = 0.01F;  // Attitude prediction covariance

static const float P_BIAS = 0.01F;  // Bias prediction covariance

/*----------------------------------------------------------------------------*/
// Earth constants, semi-axis and eccentricity
static constexpr float a  = 6378137;       // [m]
static constexpr float a2 = a * a;         // [m^2]
static constexpr float b  = 6356752.3142;  // [m]
static constexpr float b2 = b * b;         // [m^2]
static constexpr float e2 = 1 - b2 / a2;

/*----------------------------------------------------------------------------*/

// Equirectangular projection for gps: https://bit.ly/2RaMbD5

// static const float RAD =
//     6371.0F * powf(10, 3);  // [m]
//                             // Earth radius, used for the GPS correction

// static const float PHI1 =
//     0.0F;  // [rad]
//            // Latitude where the scale of the projection is true

// static const float CLAT = cosf(PHI1);

static const float SATS_NUM =
    6.0F;  // Number of available satellites (updated at runtime)

static const unsigned int JAMMING_FACTOR = 3;

#ifdef EUROC
// static const float LAT0 =
//     39.390278F;  // [deg]
//                  // Latitude of the launch location
//                  // (campo militar santa margarida)
// static const float LON0 =
//     -8.289167;  // [deg]
//                  // Longitude of the launch location
//                  // (campo militar santa margarida)

static const float EMF = 44.24F;  // [uT] micro Tesla
                                  // Earth magnetic field, used to
                                  // check if there's magnetic jamming

static const Eigen::Vector3f NED_MAG(
    0.5923F, -0.0175F,
    0.8055F);  // Normalized magnetic field vector at Ponte de Sor
               // Measurement units are not important since it's normalized
#else
// static const float LAT0 =
//     41.8098571;  // [deg]
//                  // Latitude of the launch location (roccaraso)
// static const float LON0 =
//     14.0545127;  // [deg]
//                  // Longitude of the launch location (roccaraso)

static const float EMF = 46.77F;  // [uT] micro Tesla
                                  // Earth magnetic field, used to
                                  // check if there's magnetic jamming

static const Eigen::Vector3f NED_MAG(
    0.524848, 0.035602,
    0.850451);  // Normalized magnetic field vector at Roccaraso
#endif

// normalized magentic field at Milano
// static const Eigen::Vector3f NED_MAG(0.4742, 0.025, 0.8801);

// DIMENSIONS OF MATRICES AND VECTORS

static const uint16_t N  = 13;     // State vector elements, N x 1
static const uint16_t NP = N - 1;  // P matrix, N-1 x N-1.
                                   // Reduced order thanks to the MEKF
static const uint16_t NATT = 7;    // Number of attitude related elements.
                                   // Quaternion components and biases:
                                   // [q1, q2, q3, q4, bx, by, bz]
static const uint16_t NL =
    N - NATT;  // Number of linear elements in the state
               // vector. Position and velocity:
               // [p_north, p_east, p_down, v_n, v_e, v_d]

static const uint16_t NBAR = 1;   // States of the barometer
                                  // [pressure]
static const uint16_t NGPS = 4;   // States of the gps
                                  // [lon, lat, v_north, v_east]
static const uint16_t NMAG = 3;   // States of the magnetometer
                                  // [mx, my, mz]
static const uint16_t NMEKF = 6;  // Dimension used in the MEKF.
                                  // 4 quaternion components + 3 biases - 1
                                  // The MEKF structure allows us to perform
                                  // the dimensionality reduction of 1

}  // namespace NASConfigs

}  // namespace DeathStackBoard
