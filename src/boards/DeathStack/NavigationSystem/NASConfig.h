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

#include "Eigen/Dense"

using namespace Eigen;

namespace DeathStackBoard
{

namespace NASConfigs
{

static const unsigned int NAS_STACK_SIZE = 4096;

static const float SAMPLE_RATE = 100.0F;  // [Hz]

static const float T = 1.0F / SAMPLE_RATE;  // [s]

static const uint32_t CALIBRATION_N_SAMPLES = 200;  // 1200;

static const float SIGMA_BETA =
    (float)1e-2;  // [mdps^2]
                  // Estimated gyroscope bias variance

static const float SIGMA_MAG = 0.5F;  // [uT^2]
                                      // Estimated magnetometer variance

static const float SIGMA_W = 0.5F;  // Estimated gyroscope variance

static const float SIGMA_GPS = 2.0F;  // [deg^2]
                                      // Estimated GPS variance

static const float SIGMA_BAR = 4.0F;  // [mbar^2]
                                      // Estimated barometer variance

static const float SIGMA_POS =
    1.0F;  // [m^2]
           // Estimated variance of the position noise

static const float SIGMA_VEL =
    0.1F;  // [(m/s)^2]
           // Estimated variance of the velocity noise

static const float P_POS = 0.01F;  // Position prediction covariance

static const float P_VEL = 0.01F;  // Velocity prediction covariance

static const float P_ATT = 0.01F;  // Attitude prediction covariance

static const float P_BIAS = 0.01F;  // Bias prediction covariance

/*----------------------------------------------------------------------------*/

static const float EMF = 45.0F;  // [uT] micro Tesla
                                 // Earth magnetic field, used to
                                 // check if there's magnetic jamming

// Equirectangular projection for gps: https://bit.ly/2RaMbD5

static const float RAD =
    6371.0F * powf(10, 3);  // [m]
                            // Earth radius, used for the GPS correction

static const float LAT0 = 0.0F;  // [deg]
                                 // Latitude of the launch location

static const float LON0 = 1.81e-14;  // [deg]
                                     // Longitude of the launch location

static const float PHI1 =
    0.0F;  // [rad]
           // Latitude where the scale of the projection is true

static const float CLAT = cosf(PHI1);

static const float SATS = 6.0F;  // Number of available satellites

static const float T0 = 288.15F;  // [K]
                                  // Ground temperature at the launch location

static const Vector3f NED_MAG(
    0.5969F, -0.0139F,
    0.8022F);  // Normalized magnetic field vector at the launch site
               // Measurement units are not important since it's normalized

static const float P0 = 101325.0F;  // [Pa]
                                    // Sea level pressure

// DIMENSIONS OF MATRICES AND VECTORS

static const uint16_t n = 13;  // State vector elements, n x 1
static const uint16_t np =
    n - 1;  // P matrix, n-1 x n-1. Reduced order thanks to the MEKF
static const uint16_t nl =
    n - 7;  // Number of linear elements in the state vector. -7 because we're
            // removing the 4 quaternion components and the 3 biases

}  // namespace NASConfigs

}  // namespace DeathStackBoard