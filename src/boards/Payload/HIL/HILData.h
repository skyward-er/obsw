/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <Payload/Configs/HILSimulationConfig.h>
#include <sensors/HILSimulatorData.h>

// NAS
#include <Payload/StateMachines/NASController/NASControllerData.h>
#include <algorithms/NAS/NASState.h>

// WingController
#include <Payload/StateMachines/WingController/WingControllerData.h>

namespace Payload
{

// Sensors Data
using PayloadAccelerometerSimulatorData =
    Boardcore::AccelerometerSimulatorData<Config::HIL::N_DATA_ACCEL>;
using PayloadGyroscopeSimulatorData =
    Boardcore::GyroscopeSimulatorData<Config::HIL::N_DATA_GYRO>;
using PayloadMagnetometerSimulatorData =
    Boardcore::MagnetometerSimulatorData<Config::HIL::N_DATA_MAGNETO>;
using PayloadGPSSimulatorData =
    Boardcore::GPSSimulatorData<Config::HIL::N_DATA_GPS>;
using PayloadDigitalBarometerSimulatorData =
    Boardcore::BarometerSimulatorData<Config::HIL::N_DATA_BARO_STATIC>;
using PayloadAnalogBarometerSimulatorData =
    Boardcore::BarometerSimulatorData<Config::HIL::N_DATA_BARO_PITOT>;
using PayloadTemperatureSimulatorData =
    Boardcore::TemperatureSimulatorData<Config::HIL::N_DATA_TEMP>;

enum class HILSignal : int
{
    SIMULATION_STARTED      = 1,
    SIMULATION_STOPPED      = 2,
    SIMULATION_FORCE_LAUNCH = 3
};

enum class PayloadFlightPhases
{
    SIMULATION_STARTED,
    INITIALIZED,
    CALIBRATION,
    CALIBRATION_OK,
    ARMED,
    LIFTOFF_PIN_DETACHED,
    LIFTOFF,
    MOTOR_SHUTDOWN,
    AEROBRAKES,
    APOGEE,
    PARA1,
    PARA2,
    SIMULATION_STOPPED
};

/**
 * @brief NAS data sent to the simulator
 */
struct NASStateHIL
{
    float n = 0;
    float e = 0;
    float d = 0;

    // Velocity [m/s]
    float vn = 0;
    float ve = 0;
    float vd = 0;

    // Attitude as quaternion
    float qx = 0;
    float qy = 0;
    float qz = 0;
    float qw = 1;

    float updating = 0;  // Flag if apogee is detected [bool]

    NASStateHIL()
        : n(0), e(0), d(0), vn(0), ve(0), vd(0), qx(0), qy(0), qz(0), qw(0),
          updating(0)
    {
    }

    explicit NASStateHIL(const Boardcore::NASState& nasState)
        : n(nasState.n), e(nasState.e), d(nasState.d), vn(nasState.vn),
          ve(nasState.ve), vd(nasState.vd), qx(nasState.qx), qy(nasState.qy),
          qz(nasState.qz), qw(nasState.qw), updating(1.f)  // To remove updating
    {
    }

    void print()
    {
        printf(
            "n: %+.3f\n"
            "e: %+.3f\n"
            "d: %+.3f\n"
            "vn: %+.3f\n"
            "ve: %+.3f\n"
            "vd: %+.3f\n"
            "qx: %+.3f\n"
            "qy: %+.3f\n"
            "qz: %+.3f\n"
            "qw: %+.3f\n"
            "updating: %+.3f\n",
            n, e, d, vn, ve, vd, qx, qy, qz, qw, updating);
    }
};

struct ActuatorsStateHIL
{
    float parafoilLeftPercentage  = 0;
    float parafoilRightPercentage = 0;
    float cutterState             = 0;

    ActuatorsStateHIL()
        : parafoilLeftPercentage(0.0f), parafoilRightPercentage(0.0f),
          cutterState(0.0f)
    {
    }

    ActuatorsStateHIL(float parafoilLeftPercentage,
                      float parafoilRightPercentage, float cutterState)
        : parafoilLeftPercentage(parafoilLeftPercentage),
          parafoilRightPercentage(parafoilRightPercentage),
          cutterState(cutterState)
    {
    }

    void print()
    {
        printf(
            "parafoilLeft: %f perc\n"
            "parafoilRight: %f perc\n"
            "cutterState: %f perc\n",
            parafoilLeftPercentage * 100, parafoilRightPercentage * 100,
            cutterState * 100);
    }
};

struct GuidanceDataHIL
{
    float psiRef;
    float deltaA;
    float currentTargetN;
    float currentTargetE;

    GuidanceDataHIL(float psiRef, float deltaA, float currentTargetN,
                    float currentTargetE)
        : psiRef(psiRef), deltaA(deltaA), currentTargetN(currentTargetN),
          currentTargetE(currentTargetE)
    {
    }

    GuidanceDataHIL()
        : psiRef(0.0f), deltaA(0.0f), currentTargetN(0), currentTargetE(0)
    {
    }

    void print()
    {
        printf(
            "psiRef: %f\n"
            "deltaA: %f\n"
            "currentTargetN: %f\n"
            "currentTargetE: %f\n",
            psiRef, deltaA, currentTargetN, currentTargetE);
    }
};

/**
 * @brief Data structure used by the simulator in order to directly deserialize
 * the data received
 *
 * This structure then is accessed by sensors and other components in order to
 * get the data they need
 */
struct SimulatorData
{
    PayloadAccelerometerSimulatorData accelerometer;
    PayloadGyroscopeSimulatorData gyro;
    PayloadMagnetometerSimulatorData magnetometer;
    PayloadGPSSimulatorData gps;
    PayloadDigitalBarometerSimulatorData barometer1, barometer2, barometer3;
    PayloadAnalogBarometerSimulatorData staticPitot, dynamicPitot;
    PayloadTemperatureSimulatorData temperature;
    float signal;
};

/**
 * @brief Data strudcture expected by the simulator
 */
struct ActuatorData
{
    NASStateHIL nasState;
    ActuatorsStateHIL actuatorsState;
    GuidanceDataHIL guidanceData;
    float signal;

    ActuatorData() : nasState(), actuatorsState(), guidanceData(), signal(0) {}

    ActuatorData(const NASStateHIL& nasState,
                 const ActuatorsStateHIL& actuatorsState,
                 const GuidanceDataHIL& guidanceData, float signal)
        : nasState(nasState), actuatorsState(actuatorsState),
          guidanceData(guidanceData), signal(signal)
    {
    }

    void print()
    {
        nasState.print();
        actuatorsState.print();
        guidanceData.print();
    }
};
}  // namespace Payload