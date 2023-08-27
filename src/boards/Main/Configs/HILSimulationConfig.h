/* Copyright (c) 2020-2023 Skyward Experimental Rocketry
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

#include <Main/Buses.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <drivers/timer/TimestampTimer.h>
#include <drivers/usart/USART.h>
#include <math.h>
#include <sensors/SensorInfo.h>
#include <utils/Debug.h>
#include <utils/Stats/Stats.h>

#include <list>
#include <utils/ModuleManager/ModuleManager.hpp>

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

namespace HILConfig
{

struct SensorConfig : public Boardcore::SensorInfo
{
    SensorConfig(const std::string s, const uint32_t period)
        : Boardcore::SensorInfo{s, period, []() {}, true}
    {
    }
};

/** baudrate of connection */
const int SIM_BAUDRATE = 115200;

/** Period of simulation in milliseconds */
const int SIMULATION_PERIOD = 20;

/** sample frequency of sensor (samples/second) */
const int ACCEL_FREQ = 100;
const int GYRO_FREQ  = 100;
const int MAGN_FREQ  = 100;
const int IMU_FREQ   = 100;
const int BARO_FREQ  = 20;
const int PITOT_FREQ = 20;
const int TEMP_FREQ  = 10;
const int GPS_FREQ   = 10;

/** Number of samples per sensor at each simulator iteration */
const int N_DATA_ACCEL = static_cast<int>(
    std::ceil(static_cast<float>(ACCEL_FREQ * SIMULATION_PERIOD) / 1000.0));
const int N_DATA_GYRO = static_cast<int>(
    std::ceil(static_cast<float>(GYRO_FREQ * SIMULATION_PERIOD) / 1000.0));
const int N_DATA_MAGN = static_cast<int>(
    std::ceil(static_cast<float>(MAGN_FREQ * SIMULATION_PERIOD) / 1000.0));
const int N_DATA_IMU = static_cast<int>(
    std::ceil(static_cast<float>(IMU_FREQ * SIMULATION_PERIOD) / 1000.0));
const int N_DATA_BARO = static_cast<int>(
    std::ceil(static_cast<float>(BARO_FREQ * SIMULATION_PERIOD) / 1000.0));
const int N_DATA_PITOT = static_cast<int>(
    std::ceil(static_cast<float>(PITOT_FREQ * SIMULATION_PERIOD) / 1000.0));
const int N_DATA_GPS = static_cast<int>(
    std::ceil(static_cast<float>(GPS_FREQ * SIMULATION_PERIOD) / 1000.0));
const int N_DATA_TEMP = static_cast<int>(
    std::ceil(static_cast<float>(TEMP_FREQ * SIMULATION_PERIOD) / 1000.0));

/**
 * @brief Data structure used by the simulator in order to directly deserialize
 * the data received
 *
 * This structure then is accessed by sensors and other components in order to
 * get the data they need
 */
struct SimulatorData
{
    struct Accelerometer
    {
        float measures[N_DATA_ACCEL][3];
    } accelerometer;

    struct Gyro
    {
        float measures[N_DATA_GYRO][3];
    } gyro;

    struct Magnetometer
    {
        float measures[N_DATA_MAGN][3];
    } magnetometer;

    struct Gps
    {
        float positionMeasures[N_DATA_GPS][3];
        float velocityMeasures[N_DATA_GPS][3];
        float fix;
        float num_satellites;
    } gps;

    struct Barometer
    {
        float measures[N_DATA_BARO];
    } barometer1, barometer2, barometer3, pressureChamber;

    struct Pitot
    {
        float deltaP[N_DATA_PITOT];
        float staticPressure[N_DATA_PITOT];
    } pitot;

    struct Temperature
    {
        float measure;
    } temperature;

    struct Flags
    {
        float flag_flight;
        float flag_ascent;
        float flag_burning;
        float flag_airbrakes;
        float flag_para1;
        float flag_para2;
    } flags;

    void printAccelerometer()
    {
        TRACE("accel\n");
        for (int i = 0; i < N_DATA_ACCEL; i++)
            TRACE("%+.3f\t%+.3f\t%+.3f\n", accelerometer.measures[i][0],
                  accelerometer.measures[i][1], accelerometer.measures[i][2]);
    }

    void printGyro()
    {
        TRACE("gyro\n");
        for (int i = 0; i < N_DATA_GYRO; i++)
            TRACE("%+.3f\t%+.3f\t%+.3f\n", gyro.measures[i][0],
                  gyro.measures[i][1], gyro.measures[i][2]);
    }

    void printMagnetometer()
    {
        TRACE("magneto\n");
        for (int i = 0; i < N_DATA_MAGN; i++)
            TRACE("%+.3f\t%+.3f\t%+.3f\n", magnetometer.measures[i][0],
                  magnetometer.measures[i][1], magnetometer.measures[i][2]);
    }

    void printGPS()
    {
        TRACE("gps\n");
        TRACE("pos\n");
        for (int i = 0; i < N_DATA_GPS; i++)
            TRACE("%+.3f\t%+.3f\t%+.3f\n", gps.positionMeasures[i][0],
                  gps.positionMeasures[i][1], gps.positionMeasures[i][2]);

        TRACE("vel\n");
        for (int i = 0; i < N_DATA_GPS; i++)
            TRACE("%+.3f\t%+.3f\t%+.3f\n", gps.velocityMeasures[i][0],
                  gps.velocityMeasures[i][1], gps.velocityMeasures[i][2]);
        TRACE("fix:%+.3f\tnsat:%+.3f\n", gps.fix, gps.num_satellites);
    }

    void printBarometer1()
    {
        TRACE("press1\n");
        for (int i = 0; i < N_DATA_BARO; i++)
            TRACE("%+.3f\n", barometer1.measures[i]);
    }

    void printBarometer2()
    {
        TRACE("press2\n");
        for (int i = 0; i < N_DATA_BARO; i++)
            TRACE("%+.3f\n", barometer2.measures[i]);
    }

    void printBarometer3()
    {
        TRACE("press3\n");
        for (int i = 0; i < N_DATA_BARO; i++)
            TRACE("%+.3f\n", barometer3.measures[i]);
    }

    void printBarometerChamber()
    {
        TRACE("press3\n");
        for (int i = 0; i < N_DATA_BARO; i++)
            TRACE("%+.3f\n", pressureChamber.measures[i]);
    }

    void printPitot()
    {
        TRACE("pitot\n");
        for (int i = 0; i < N_DATA_PITOT; i++)
            TRACE("%+.3f, %+.3f\n", pitot.staticPressure[i], pitot.deltaP[i]);
    }

    void printTemperature()
    {
        TRACE("temp\n");
        for (int i = 0; i < N_DATA_TEMP; i++)
            TRACE("%+.3f\n", temperature.measure);
    }

    void printFlags()
    {
        TRACE("flags\n");
        TRACE(
            "flight:\t%+.3f\n"
            "ascent:\t%+.3f\n"
            "burning:\t%+.3f\n"
            "airbrakes:\t%+.3f\n"
            "para1:\t%+.3f\n"
            "para2:\t%+.3f\n",
            flags.flag_flight, flags.flag_ascent, flags.flag_burning,
            flags.flag_airbrakes, flags.flag_para1, flags.flag_para2);
    }

    void print()
    {
        printAccelerometer();
        printGyro();
        printMagnetometer();
        printGPS();
        printBarometer1();
        printBarometer2();
        printBarometer3();
        printBarometerChamber();
        printPitot();
        printTemperature();
        printFlags();
    }
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

    ADAStateHIL(Boardcore::ADAState adaState,
                Main::ADAControllerStatus adaStatus)
        : mslAltitude(adaState.mslAltitude), aglAltitude(adaState.aglAltitude),
          verticalSpeed(adaState.verticalSpeed),
          apogeeDetected(adaStatus.state == Main::ADAControllerState::END),
          updating(adaStatus.state == Main::ADAControllerState::ARMED ||
                   adaStatus.state == Main::ADAControllerState::SHADOW_MODE ||
                   adaStatus.state == Main::ADAControllerState::ACTIVE)
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
        : n(0), e(0), d(0), vn(0), ve(0), vd(0), qx(0), qy(0), qz(0), qw(0),
          updating(0)
    {
    }

    NASStateHIL(Boardcore::NASState adaState,
                Main::NASControllerStatus adaStatus)
        : n(adaState.n), e(adaState.e), d(adaState.d), vn(adaState.vn),
          ve(adaState.ve), vd(adaState.vd), qx(adaState.qx), qy(adaState.qy),
          qz(adaState.qz), qw(adaState.qw),
          updating(adaStatus.state == Main::NASControllerState::ACTIVE)
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

    AirBrakesStateHIL(Main::ABKControllerStatus abkStatus)
        : updating(abkStatus.state == Main::ABKControllerState::ACTIVE)
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

    MEAStateHIL(Boardcore::MEAState state, Main::MEAControllerStatus status)
        : correctedPressure(state.correctedPressure),
          estimatedMass(status.estimatedMass),
          estimatedApogee(status.estimatedApogee), updating(/* TODO update! */)
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

struct ActuatorsStateHIL
{
    float airbrakesPercentage    = 0;
    float expulsionPercentage    = 0;
    float mainValvePercentage    = 0;
    float ventingValvePercentage = 0;

    ActuatorsStateHIL()
        : airbrakesPercentage(0.0f), expulsionPercentage(0.0f),
          mainValvePercentage(0.0f), ventingValvePercentage(0.0f)
    {
    }

    ActuatorsStateHIL(float airbrakesPercentage, float expulsionPercentage,
                      float mainValvePercentage, float ventingValvePercentage)
        : airbrakesPercentage(airbrakesPercentage),
          expulsionPercentage(expulsionPercentage),
          mainValvePercentage(mainValvePercentage),
          ventingValvePercentage(ventingValvePercentage)
    {
    }

    void print()
    {
        printf(
            "airbrakes: %f perc\n"
            "expulsion: %f perc\n"
            "mainValve: %f perc\n"
            "venting: %f perc\n",
            airbrakesPercentage * 100, expulsionPercentage * 100,
            mainValvePercentage * 100, ventingValvePercentage * 100);
    }
};

struct FlagsHIL
{
    float flag_flight;
    float flag_ascent;
    float flag_burning;
    float flag_airbrakes;
    float flag_para1;
    float flag_para2;

    FlagsHIL(float flag_flight, float flag_ascent, float flag_burning,
             float flag_airbrakes, float flag_para1, float flag_para2)
        : flag_flight(flag_flight), flag_ascent(flag_ascent),
          flag_burning(flag_burning), flag_airbrakes(flag_airbrakes),
          flag_para1(flag_para1), flag_para2(flag_para2)
    {
    }

    FlagsHIL()
        : flag_flight(0.0f), flag_ascent(0.0f), flag_burning(0.0f),
          flag_airbrakes(0.0f), flag_para1(0.0f), flag_para2(0.0f)
    {
    }

    void print()
    {
        printf(
            "flag_flight: %f\n"
            "flag_ascent: %f\n"
            "flag_burning: %f\n"
            "flag_airbrakes: %f\n"
            "flag_para1: %f\n"
            "flag_para2: %f\n",
            flag_flight, flag_ascent, flag_burning, flag_airbrakes, flag_para1,
            flag_para2);
    }
};

/**
 * @brief Data strudcture expected by the simulator
 */
struct ActuatorData
{
    ADAStateHIL adaState;
    NASStateHIL nasState;
    AirBrakesStateHIL airBrakesState;
    MEAStateHIL meaState;
    ActuatorsStateHIL actuatorsState;
    FlagsHIL flags;

    ActuatorData()
        : adaState(), nasState(), airBrakesState(), meaState(),
          actuatorsState(), flags()
    {
    }

    ActuatorData(ADAStateHIL adaState, NASStateHIL nasState,
                 AirBrakesStateHIL airBrakesState, MEAStateHIL meaState,
                 ActuatorsStateHIL actuatorsState, Main::FlightModeManager* fmm)
        : adaState(adaState), nasState(nasState),
          airBrakesState(airBrakesState), meaState(meaState),
          actuatorsState(actuatorsState)
    {
        flags.flag_flight =
            (fmm->testState(&Main::FlightModeManager::state_flying) ? 1 : 0);
        flags.flag_ascent =
            (fmm->testState(&Main::FlightModeManager::state_powered_ascent) ||
                     fmm->testState(
                         &Main::FlightModeManager::state_unpowered_ascent)
                 ? 1
                 : 0);
        flags.flag_burning =
            (fmm->testState(&Main::FlightModeManager::state_powered_ascent)
                 ? 1
                 : 0);
        flags.flag_airbrakes =
            (fmm->testState(&Main::FlightModeManager::state_unpowered_ascent)
                 ? 1
                 : 0);
        flags.flag_para1 =
            (fmm->testState(&Main::FlightModeManager::state_drogue_descent)
                 ? 1
                 : 0);
        flags.flag_para2 =
            (fmm->testState(&Main::FlightModeManager::state_terminal_descent)
                 ? 1
                 : 0);
    }

    void print()
    {
        adaState.print();
        nasState.print();
        airBrakesState.print();
        meaState.print();
        actuatorsState.print();
        flags.print();
    }
};

}  // namespace HILConfig