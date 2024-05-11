/* Copyright (c) 2023-2024 Skyward Experimental Rocketry
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
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <drivers/usart/USART.h>
#include <events/EventBroker.h>
#include <math.h>
#include <sensors/HILSensors/IncludeHILSensors.h>
#include <sensors/SensorInfo.h>
#include <utils/Debug.h>
#include <utils/Stats/Stats.h>

#include <list>
#include <utils/ModuleManager/ModuleManager.hpp>

#include "SensorsConfig.h"

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

constexpr bool IS_FULL_HIL = true;

/** Period of simulation in milliseconds */
constexpr int SIMULATION_PERIOD = 100;
constexpr int SIMULATION_FREQ   = 1000 / SIMULATION_PERIOD;

/** sample frequency of sensor (samples/second) */
constexpr int ACCEL_FREQ        = 1000 / Main::SensorsConfig::LSM6DSRX_PERIOD;
constexpr int GYRO_FREQ         = 1000 / Main::SensorsConfig::LSM6DSRX_PERIOD;
constexpr int MAGN_FREQ         = 1000 / Main::SensorsConfig::LIS2MDL_PERIOD;
constexpr int IMU_FREQ          = 1000 / Main::SensorsConfig::IMU_PERIOD;
constexpr int ANALOG_BARO_FREQ  = 1000 / Main::SensorsConfig::ADS131M08_PERIOD;
constexpr int DIGITAL_BARO_FREQ = 1000 / Main::SensorsConfig::LPS22DF_PERIOD;
constexpr int BARO_CHAMBER_FREQ = 50;
constexpr int PITOT_FREQ        = 20;
constexpr int TEMP_FREQ = 1000 / SIMULATION_PERIOD;  // One sample per iteration
// Hardcoded to 10 Hz so that we sample at least 1 time every integration step
constexpr int GPS_FREQ = 1000 / Main::SensorsConfig::UBXGPS_PERIOD;

static_assert((ACCEL_FREQ % SIMULATION_FREQ) == 0,
              "N_DATA_ACCEL not an integer");
static_assert((GYRO_FREQ % SIMULATION_FREQ) == 0, "N_DATA_GYRO not an integer");
static_assert((MAGN_FREQ % SIMULATION_FREQ) == 0, "N_DATA_MAGN not an integer");
static_assert((IMU_FREQ % SIMULATION_FREQ) == 0, "N_DATA_IMU not an integer");
static_assert((ANALOG_BARO_FREQ % SIMULATION_FREQ) == 0,
              "N_DATA_ANALOG_BARO not an integer");
static_assert((DIGITAL_BARO_FREQ % SIMULATION_FREQ) == 0,
              "N_DATA_DIGITAL_BARO not an integer");
static_assert((BARO_CHAMBER_FREQ % SIMULATION_FREQ) == 0,
              "N_DATA_BARO_CHAMBER not an integer");
static_assert((PITOT_FREQ % SIMULATION_FREQ) == 0,
              "N_DATA_PITOT not an integer");
static_assert((TEMP_FREQ % SIMULATION_FREQ) == 0, "N_DATA_TEMP not an integer");
static_assert((GPS_FREQ % SIMULATION_FREQ) == 0, "N_DATA_GPS not an integer");

/** Number of samples per sensor at each simulator iteration */
constexpr int N_DATA_ACCEL =
    static_cast<int>((ACCEL_FREQ * SIMULATION_PERIOD) / 1000.0);
constexpr int N_DATA_GYRO =
    static_cast<int>((GYRO_FREQ * SIMULATION_PERIOD) / 1000.0);
constexpr int N_DATA_MAGNETO =
    static_cast<int>((MAGN_FREQ * SIMULATION_PERIOD) / 1000.0);
constexpr int N_DATA_IMU =
    static_cast<int>((IMU_FREQ * SIMULATION_PERIOD) / 1000.0);
constexpr int N_DATA_ANALOG_BARO =
    static_cast<int>((ANALOG_BARO_FREQ * SIMULATION_PERIOD) / 1000.0);
constexpr int N_DATA_DIGITAL_BARO =
    static_cast<int>((DIGITAL_BARO_FREQ * SIMULATION_PERIOD) / 1000.0);
constexpr int N_DATA_BARO_CHAMBER =
    static_cast<int>((BARO_CHAMBER_FREQ * SIMULATION_PERIOD) / 1000.0);
constexpr int N_DATA_PITOT =
    static_cast<int>((PITOT_FREQ * SIMULATION_PERIOD) / 1000.0);
constexpr int N_DATA_GPS =
    static_cast<int>((GPS_FREQ * SIMULATION_PERIOD) / 1000.0);
constexpr int N_DATA_TEMP =
    static_cast<int>((TEMP_FREQ * SIMULATION_PERIOD) / 1000.0);

// Sensors Data
using MainAccelerometerSimulatorData = AccelerometerSimulatorData<N_DATA_ACCEL>;
using MainGyroscopeSimulatorData     = GyroscopeSimulatorData<N_DATA_GYRO>;
using MainMagnetometerSimulatorData = MagnetometerSimulatorData<N_DATA_MAGNETO>;
using MainGPSSimulatorData          = GPSSimulatorData<N_DATA_GPS>;
using MainBarometerSimulatorData = BarometerSimulatorData<N_DATA_DIGITAL_BARO>;
using MainChamberPressureSimulatorData =
    BarometerSimulatorData<N_DATA_BARO_CHAMBER>;
using MainPitotSimulatorData       = PitotSimulatorData<N_DATA_PITOT>;
using MainTemperatureSimulatorData = TemperatureSimulatorData<N_DATA_TEMP>;

// Sensors
using MainHILAccelerometer    = HILAccelerometer<N_DATA_ACCEL>;
using MainHILGyroscope        = HILGyroscope<N_DATA_GYRO>;
using MainHILMagnetometer     = HILMagnetometer<N_DATA_MAGNETO>;
using MainHILGps              = HILGps<N_DATA_GPS>;
using MainHILBarometer        = HILBarometer<N_DATA_DIGITAL_BARO>;
using MainHILChamberBarometer = HILBarometer<N_DATA_BARO_CHAMBER>;
using MainHILPitot            = HILPitot<N_DATA_PITOT>;
using MainHILTemperature      = HILTemperature<N_DATA_TEMP>;

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
        : n(0), e(0), d(0), vn(0), ve(0), vd(0), qx(0), qy(0), qz(0), qw(1),
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
    MainAccelerometerSimulatorData accelerometer;
    MainGyroscopeSimulatorData gyro;
    MainMagnetometerSimulatorData magnetometer;
    MainGPSSimulatorData gps;
    MainBarometerSimulatorData barometer1, barometer2, barometer3;
    MainChamberPressureSimulatorData pressureChamber;
    MainPitotSimulatorData pitot;
    MainTemperatureSimulatorData temperature;
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
            (fmm->testState(&Main::FlightModeManager::state_powered_ascent) ||
                     fmm->testState(
                         &Main::FlightModeManager::state_unpowered_ascent) ||
                     fmm->testState(
                         &Main::FlightModeManager::state_drogue_descent) ||
                     fmm->testState(
                         &Main::FlightModeManager::state_terminal_descent)
                 ? 1
                 : 0);

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

enum MainFlightPhases
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
    AEROBRAKES,
    APOGEE,
    PARA1,
    PARA2,
    SIMULATION_STOPPED
};

using MainHILTransceiver =
    HILTransceiver<MainFlightPhases, SimulatorData, ActuatorData>;
using MainHIL = HIL<MainFlightPhases, SimulatorData, ActuatorData>;

class MainHILPhasesManager
    : public HILPhasesManager<MainFlightPhases, SimulatorData, ActuatorData>
{
public:
    explicit MainHILPhasesManager(
        std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition)
        : HILPhasesManager<MainFlightPhases, SimulatorData, ActuatorData>(
              getCurrentPosition)
    {
        flagsFlightPhases = {{MainFlightPhases::SIM_FLYING, false},
                             {MainFlightPhases::SIM_ASCENT, false},
                             {MainFlightPhases::SIM_BURNING, false},
                             {MainFlightPhases::SIM_AEROBRAKES, false},
                             {MainFlightPhases::SIM_PARA1, false},
                             {MainFlightPhases::SIM_PARA2, false},
                             {MainFlightPhases::SIMULATION_STARTED, false},
                             {MainFlightPhases::CALIBRATION, false},
                             {MainFlightPhases::CALIBRATION_OK, false},
                             {MainFlightPhases::ARMED, false},
                             {MainFlightPhases::LIFTOFF_PIN_DETACHED, false},
                             {MainFlightPhases::LIFTOFF, false},
                             {MainFlightPhases::AEROBRAKES, false},
                             {MainFlightPhases::APOGEE, false},
                             {MainFlightPhases::PARA1, false},
                             {MainFlightPhases::PARA2, false},
                             {MainFlightPhases::SIMULATION_STOPPED, false}};

        prev_flagsFlightPhases = flagsFlightPhases;

        auto& eventBroker = Boardcore::EventBroker::getInstance();
        eventBroker.subscribe(this, Common::TOPIC_ABK);
        eventBroker.subscribe(this, Common::TOPIC_ADA);
        eventBroker.subscribe(this, Common::TOPIC_MEA);
        eventBroker.subscribe(this, Common::TOPIC_DPL);
        eventBroker.subscribe(this, Common::TOPIC_CAN);
        eventBroker.subscribe(this, Common::TOPIC_FLIGHT);
        eventBroker.subscribe(this, Common::TOPIC_FMM);
        eventBroker.subscribe(this, Common::TOPIC_FSR);
        eventBroker.subscribe(this, Common::TOPIC_NAS);
        eventBroker.subscribe(this, Common::TOPIC_TMTC);
        eventBroker.subscribe(this, Common::TOPIC_MOTOR);
        eventBroker.subscribe(this, Common::TOPIC_TARS);
        eventBroker.subscribe(this, Common::TOPIC_ALT);
    }

    void processFlags(const SimulatorData& simulatorData) override
    {
        std::vector<MainFlightPhases> changed_flags;

        // set true when the first packet from the simulator arrives
        if (isSetTrue(MainFlightPhases::SIMULATION_STARTED))
        {
            t_start = Boardcore::TimestampTimer::getTimestamp();

            TRACE("[HIL] ------- SIMULATION STARTED ! ------- \n");
            changed_flags.push_back(MainFlightPhases::SIMULATION_STARTED);
        }

        if (flagsFlightPhases[MainFlightPhases::SIM_FLYING])
        {
            if (isSetTrue(MainFlightPhases::SIM_FLYING))
            {
                registerOutcomes(MainFlightPhases::SIM_FLYING);
                TRACE("[HIL] ------- SIMULATOR LIFTOFF ! ------- \n");
                changed_flags.push_back(MainFlightPhases::SIM_FLYING);
            }
        }

        /* calling the callbacks subscribed to the changed flags */
        for (unsigned int i = 0; i < changed_flags.size(); i++)
        {
            std::vector<TCallback> callbacksToCall =
                callbacks[changed_flags[i]];
            for (unsigned int j = 0; j < callbacksToCall.size(); j++)
            {
                callbacksToCall[j]();
            }
        }

        prev_flagsFlightPhases = flagsFlightPhases;
    }

    void printOutcomes()
    {
        printf("OUTCOMES: (times dt from liftoff)\n\n");
        printf("Simulation time: %.3f [sec]\n\n",
               (double)(t_stop - t_start) / 1000000.0f);

        printf("Motor stopped burning (simulation flag): \n");
        outcomes[MainFlightPhases::SIM_BURNING].print(t_liftoff);

        printf("Airbrakes exit shadowmode: \n");
        outcomes[MainFlightPhases::AEROBRAKES].print(t_liftoff);

        printf("Apogee: \n");
        outcomes[MainFlightPhases::APOGEE].print(t_liftoff);

        printf("Parachute 1: \n");
        outcomes[MainFlightPhases::PARA1].print(t_liftoff);

        printf("Parachute 2: \n");
        outcomes[MainFlightPhases::PARA2].print(t_liftoff);

        printf("Simulation Stopped: \n");
        outcomes[MainFlightPhases::SIMULATION_STOPPED].print(t_liftoff);

        // auto cpuMeter = Boardcore::CpuMeter::getCpuStats();
        // printf("max cpu usage: %+.3f\n", cpuMeter.maxValue);
        // printf("avg cpu usage: %+.3f\n", cpuMeter.mean);
        // printf("min free heap: %+.3lu\n", cpuMeter.minFreeHeap);
        // printf("max free stack:%+.3lu\n", cpuMeter.minFreeStack);
    }

private:
    void handleEvent(const Boardcore::Event& e) override
    {
        std::vector<MainFlightPhases> changed_flags;
        switch (e)
        {
            case Common::Events::FMM_INIT_ERROR:
                printf("[HIL] ------- INIT FAILED ! ------- \n");
            case Common::Events::FMM_INIT_OK:
                setFlagFlightPhase(MainFlightPhases::CALIBRATION, true);
                TRACE("[HIL] ------- CALIBRATION ! ------- \n");
                changed_flags.push_back(MainFlightPhases::CALIBRATION);
                break;
            case Common::Events::FLIGHT_DISARMED:
                setFlagFlightPhase(MainFlightPhases::CALIBRATION_OK, true);
                TRACE("[HIL] CALIBRATION OK!\n");
                changed_flags.push_back(MainFlightPhases::CALIBRATION_OK);
                break;
            case Common::Events::FLIGHT_ARMED:
                setFlagFlightPhase(MainFlightPhases::ARMED, true);
                printf("[HIL] ------- READY TO LAUNCH ! ------- \n");
                changed_flags.push_back(MainFlightPhases::ARMED);
                break;
            case Common::Events::FLIGHT_LAUNCH_PIN_DETACHED:
                setFlagFlightPhase(MainFlightPhases::LIFTOFF_PIN_DETACHED,
                                   true);
                TRACE("[HIL] ------- LIFTOFF PIN DETACHED ! ------- \n");
                changed_flags.push_back(MainFlightPhases::LIFTOFF_PIN_DETACHED);
                break;
            case Common::Events::FLIGHT_LIFTOFF:
            case Common::Events::TMTC_FORCE_LAUNCH:
                t_liftoff = Boardcore::TimestampTimer::getTimestamp();
                printf("[HIL] ------- LIFTOFF -------: %f, %f \n",
                       getCurrentPosition().z, getCurrentPosition().vz);
                changed_flags.push_back(MainFlightPhases::LIFTOFF);
                break;
            case Common::Events::ABK_SHADOW_MODE_TIMEOUT:
                setFlagFlightPhase(MainFlightPhases::AEROBRAKES, true);
                registerOutcomes(MainFlightPhases::AEROBRAKES);
                TRACE("[HIL] ABK shadow mode timeout\n");
                changed_flags.push_back(MainFlightPhases::AEROBRAKES);
                break;
            case Common::Events::ADA_SHADOW_MODE_TIMEOUT:
                TRACE("[HIL] ADA shadow mode timeout\n");
                break;
            case Common::Events::ABK_DISABLE:
                setFlagFlightPhase(MainFlightPhases::AEROBRAKES, false);
                TRACE("[HIL] ABK disabled\n");
                break;
            case Common::Events::FLIGHT_APOGEE_DETECTED:
            case Common::Events::CAN_APOGEE_DETECTED:
                setFlagFlightPhase(MainFlightPhases::AEROBRAKES, false);
                registerOutcomes(MainFlightPhases::APOGEE);
                printf("[HIL] ------- APOGEE DETECTED ! ------- %f, %f \n",
                       getCurrentPosition().z, getCurrentPosition().vz);
                changed_flags.push_back(MainFlightPhases::APOGEE);
                break;
            case Common::Events::FLIGHT_DROGUE_DESCENT:
            case Common::Events::TMTC_FORCE_EXPULSION:
                setFlagFlightPhase(MainFlightPhases::PARA1, true);
                registerOutcomes(MainFlightPhases::PARA1);
                printf("[HIL] ------- PARA1 ! -------%f, %f \n",
                       getCurrentPosition().z, getCurrentPosition().vz);
                changed_flags.push_back(MainFlightPhases::PARA1);
                break;
            case Common::Events::FLIGHT_WING_DESCENT:
            case Common::Events::FLIGHT_DPL_ALT_DETECTED:
            case Common::Events::TMTC_FORCE_DEPLOYMENT:
                setFlagFlightPhase(MainFlightPhases::PARA1, false);
                setFlagFlightPhase(MainFlightPhases::PARA2, true);
                registerOutcomes(MainFlightPhases::PARA2);
                printf("[HIL] ------- PARA2 ! ------- %f, %f \n",
                       getCurrentPosition().z, getCurrentPosition().vz);
                changed_flags.push_back(MainFlightPhases::PARA2);
                break;
            case Common::Events::FLIGHT_LANDING_DETECTED:
            case Common::Events::TMTC_FORCE_LANDING:
                t_stop = Boardcore::TimestampTimer::getTimestamp();
                setFlagFlightPhase(MainFlightPhases::PARA2, false);
                setFlagFlightPhase(MainFlightPhases::SIMULATION_STOPPED, true);
                changed_flags.push_back(MainFlightPhases::SIMULATION_STOPPED);
                registerOutcomes(MainFlightPhases::SIMULATION_STOPPED);
                TRACE("[HIL] ------- SIMULATION STOPPED ! -------: %f \n\n\n",
                      (double)t_stop / 1000000.0f);
                printOutcomes();
                break;
            default:
                TRACE("%s event\n", Common::getEventString(e).c_str());
        }

        /* calling the callbacks subscribed to the changed flags */
        for (unsigned int i = 0; i < changed_flags.size(); i++)
        {
            std::vector<TCallback> callbacksToCall =
                callbacks[changed_flags[i]];
            for (unsigned int j = 0; j < callbacksToCall.size(); j++)
            {
                callbacksToCall[j]();
            }
        }

        prev_flagsFlightPhases = flagsFlightPhases;
    }
};

}  // namespace HILConfig