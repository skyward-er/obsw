/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Mozzarelli
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

namespace DeathStackBoard
{
namespace ADA
{
// All possible states of the ADA FMM
enum class ADAState {
    UNDEFINED,
    CALIBRATING,
    IDLE,
    SHADOW_MODE,
    ACTIVE,
    FIRST_DESCENT_PHASE,
    END
};

// Struct to log apogee detection
struct ApogeeDetected {
    ADAState state;
    long long tick;
};

// Struct to log current state
struct ADAStatus
{
    long long timestamp;
    ADAState state = ADAState::UNDEFINED;

    ApogeeDetected last_apogee;

    long long last_dpl_pressure_tick;
};

struct KalmanState
{
    float x0;
    float x1;
    float x2;
};

struct TargetDeploymentPressure
{
    uint16_t deployment_pressure;
};

// Struct of calibration data
struct ADACalibrationData {
    float   var        = 0.0;      // Sample variance
    int     n_samples  = 0;        // Number of samples collected
    float   avg        = 0.0;      // Average pressure
};



}
}