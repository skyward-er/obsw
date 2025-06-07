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

#include <Main/Configs/HILSimulationConfig.h>
#include <sensors/HILSimulatorData.h>

// ADA
#include <Main/StateMachines/ADAController/ADAControllerData.h>
#include <algorithms/ADA/ADAData.h>

// NAS
#include <Main/StateMachines/NASController/NASControllerData.h>
#include <algorithms/NAS/NASState.h>

// ABK
#include <Main/StateMachines/ABKController/ABKControllerData.h>
#include <algorithms/AirBrakes/AirBrakesInterp.h>

// MEA
#include <Main/StateMachines/MEAController/MEAControllerData.h>
#include <algorithms/MEA/MEAData.h>

namespace Main
{

// Sensors Data
using MainAccelerometerSimulatorData =
    Boardcore::AccelerometerSimulatorData<Config::HIL::N_DATA_ACCEL>;
using MainGyroscopeSimulatorData =
    Boardcore::GyroscopeSimulatorData<Config::HIL::N_DATA_GYRO>;
using MainMagnetometerSimulatorData =
    Boardcore::MagnetometerSimulatorData<Config::HIL::N_DATA_MAGNETO>;
using MainGPSSimulatorData =
    Boardcore::GPSSimulatorData<Config::HIL::N_DATA_GPS>;
using MainBarometerSimulatorData =
    Boardcore::BarometerSimulatorData<Config::HIL::N_DATA_BARO_STATIC>;
using MainChamberPressureSimulatorData =
    Boardcore::BarometerSimulatorData<Config::HIL::N_DATA_BARO_CHAMBER>;
using MainPitotSimulatorData =
    Boardcore::PitotSimulatorData<Config::HIL::N_DATA_PITOT>;
using MainTemperatureSimulatorData =
    Boardcore::TemperatureSimulatorData<Config::HIL::N_DATA_TEMP>;

enum class HILSignal : int
{
    SIMULATION_STARTED      = 1,
    SIMULATION_STOPPED      = 2,
    SIMULATION_FORCE_LAUNCH = 3
};

enum class MainFlightPhases
{
    SIM_FLYING,
    SIM_ASCENT,
    SIM_BURNING,
    SIM_AEROBRAKES,
    SIM_PARA1,
    SIM_PARA2,
    SIMULATION_STARTED,
    CALIBRATION,
    CALIBRATION_OK,
    ARMED,
    LIFTOFF_PIN_DETACHED,
    LIFTOFF,
    SHUTDOWN,
    AEROBRAKES,
    APOGEE,
    PARA1,
    PARA2,
    SIMULATION_STOPPED
};

/**
 * @brief ADA data sent to the simulator
 */
struct ADAStateHIL
{
    float mslAltitude    = 0;  // Altitude at mean sea level [m].
    float aglAltitude    = 0;  // Altitude above ground level [m].
    float verticalSpeed  = 0;  // Vertical speed [m/s].
    float apogeeDetected = 0;  // Flag if apogee is detected [bool]
    float updating       = 0;  //

    ADAStateHIL()
        : mslAltitude(0), aglAltitude(0), verticalSpeed(0), apogeeDetected(0),
          updating(0)
    {
    }

    ADAStateHIL(const Boardcore::ADAState& adaState,
                const Main::ADAControllerState& state)
        : mslAltitude(adaState.mslAltitude), aglAltitude(adaState.aglAltitude),
          verticalSpeed(adaState.verticalSpeed),
          apogeeDetected(state >= ADAControllerState::ACTIVE_DROGUE_DESCENT),
          updating(state >= ADAControllerState::ARMED)
    {
    }

    void print()
    {
        printf(
            "mslAltitude: %+.3f\n"
            "aglAltitude: %+.3f\n"
            "verticalSpeed: %+.3f\n"
            "apogeeDetected: %+.3f\n"
            "updating: %+.3f\n",
            mslAltitude, aglAltitude, verticalSpeed, apogeeDetected, updating);
    }
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
        : n(0), e(0), d(0), vn(0), ve(0), vd(0), qx(0), qy(0), qz(0), qw(1),
          updating(0)
    {
    }

    NASStateHIL(const Boardcore::NASState& nasState,
                const Main::NASControllerState& state)
        : n(nasState.n), e(nasState.e), d(nasState.d), vn(nasState.vn),
          ve(nasState.ve), vd(nasState.vd), qx(nasState.qx), qy(nasState.qy),
          qz(nasState.qz), qw(nasState.qw),
          updating(state >= Main::NASControllerState::ACTIVE)
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

/**
 * @brief ABK data sent to the simulator
 */
struct AirBrakesStateHIL
{
    float updating = 0;  // Flag if apogee is detected [bool]

    AirBrakesStateHIL() : updating(0) {}

    explicit AirBrakesStateHIL(const Main::ABKControllerState& state)
        : updating(state >= Main::ABKControllerState::ACTIVE)
    {
    }

    void print() { printf("updating: %+.3f\n", updating); }
};

/**
 * @brief MEA data sent to the simulator
 */
struct MEAStateHIL
{
    float correctedPressure = 0;

    float estimatedMass   = 0;
    float estimatedApogee = 0;

    float updating = 0;  // Flag if apogee is detected [bool]

    MEAStateHIL()
        : correctedPressure(0), estimatedMass(0), estimatedApogee(0),
          updating(0)
    {
    }

    MEAStateHIL(const Boardcore::MEAState& meaState,
                const Main::MEAControllerState& state)
        : correctedPressure(meaState.estimatedPressure),
          estimatedMass(meaState.estimatedMass),
          estimatedApogee(meaState.estimatedApogee),
          updating(state >= Main::MEAControllerState::ARMED)
    {
    }

    void print()
    {
        printf(
            "correctedPressure: %+.3f\n"
            "estimatedMass: %+.3f\n"
            "estimatedApogee: %+.3f\n"
            "updating: %+.3f\n",
            correctedPressure, estimatedMass, estimatedApogee, updating);
    }
};

/**
 * @brief Actuators data sent to the simulator
 */
struct ActuatorsStateHIL
{
    float airbrakesPercentage    = 0;
    float expulsionPercentage    = 0;
    float mainValvePercentage    = 0;
    float ventingValvePercentage = 0;
    float cutterState            = 0;

    ActuatorsStateHIL()
        : airbrakesPercentage(0.0f), expulsionPercentage(0.0f),
          mainValvePercentage(0.0f), ventingValvePercentage(0.0f)
    {
    }

    ActuatorsStateHIL(float airbrakesPercentage, float expulsionPercentage,
                      float mainValvePercentage, float ventingValvePercentage,
                      float cutterState)
        : airbrakesPercentage(airbrakesPercentage),
          expulsionPercentage(expulsionPercentage),
          mainValvePercentage(mainValvePercentage),
          ventingValvePercentage(ventingValvePercentage),
          cutterState(cutterState)
    {
    }

    void print()
    {
        printf(
            "airbrakes: %f perc\n"
            "expulsion: %f perc\n"
            "mainValve: %f perc\n"
            "venting: %f perc\n"
            "cutter: %f\n",
            airbrakesPercentage * 100, expulsionPercentage * 100,
            mainValvePercentage * 100, ventingValvePercentage * 100,
            cutterState);
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
    MainAccelerometerSimulatorData accelerometer1, accelerometer2;
    MainGyroscopeSimulatorData gyro1, gyro2;
    MainMagnetometerSimulatorData magnetometer;
    MainGPSSimulatorData gps;
    MainBarometerSimulatorData barometer1, barometer2, barometer3;
    MainChamberPressureSimulatorData pressureChamber;
    MainPitotSimulatorData pitot;
    MainTemperatureSimulatorData temperature;
    float signal;
};

/**
 * @brief Data structure expected by the simulator
 */
struct ActuatorData
{
    ADAStateHIL adaState;
    NASStateHIL nasState;
    AirBrakesStateHIL airBrakesState;
    MEAStateHIL meaState;
    ActuatorsStateHIL actuatorsState;

    ActuatorData()
        : adaState(), nasState(), airBrakesState(), meaState(), actuatorsState()
    {
    }

    ActuatorData(const ADAStateHIL& adaState, const NASStateHIL& nasState,
                 const AirBrakesStateHIL& airBrakesState,
                 const MEAStateHIL& meaState,
                 const ActuatorsStateHIL& actuatorsState)
        : adaState(adaState), nasState(nasState),
          airBrakesState(airBrakesState), meaState(meaState),
          actuatorsState(actuatorsState)
    {
    }

    void print()
    {
        adaState.print();
        nasState.print();
        airBrakesState.print();
        meaState.print();
        actuatorsState.print();
    }
};

}  // namespace Main
