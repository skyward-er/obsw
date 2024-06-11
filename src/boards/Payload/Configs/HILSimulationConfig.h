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

#include <Payload/Buses.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <drivers/usart/USART.h>
#include <events/EventBroker.h>
#include <hil/HIL.h>
#include <math.h>
#include <sensors/HILSimulatorData.h>
#include <sensors/SensorInfo.h>
#include <utils/Debug.h>
#include <utils/Stats/Stats.h>

#include <list>
#include <utils/ModuleManager/ModuleManager.hpp>

#include "SensorsConfig.h"

// NAS
#include <Payload/StateMachines/NASController/NASControllerData.h>
#include <algorithms/NAS/NASState.h>

// WingController
#include <Payload/StateMachines/WingController/WingControllerData.h>

namespace HILConfig
{

/** Period of simulation in milliseconds */
constexpr int SIMULATION_PERIOD = 100;

/** sample frequency of sensor (samples/second) */
constexpr int ACCEL_PERIOD        = Payload::SensorsConfig::LSM6DSRX_PERIOD;
constexpr int GYRO_PERIOD         = Payload::SensorsConfig::LSM6DSRX_PERIOD;
constexpr int MAGN_PERIOD         = Payload::SensorsConfig::LIS2MDL_PERIOD;
constexpr int IMU_PERIOD          = Payload::SensorsConfig::IMU_PERIOD;
constexpr int ANALOG_BARO_PERIOD  = Payload::SensorsConfig::ADS131M08_PERIOD;
constexpr int DIGITAL_BARO_PERIOD = Payload::SensorsConfig::LPS22DF_PERIOD;
constexpr int BARO_CHAMBER_PERIOD = 20;
constexpr int TEMP_PERIOD         = SIMULATION_PERIOD;  // One sample
constexpr int GPS_PERIOD          = Payload::SensorsConfig::UBXGPS_PERIOD;

static_assert((SIMULATION_PERIOD % ACCEL_PERIOD) == 0,
              "N_DATA_ACCEL not an integer");
static_assert((SIMULATION_PERIOD % GYRO_PERIOD) == 0,
              "N_DATA_GYRO not an integer");
static_assert((SIMULATION_PERIOD % MAGN_PERIOD) == 0,
              "N_DATA_MAGN not an integer");
static_assert((SIMULATION_PERIOD % IMU_PERIOD) == 0,
              "N_DATA_IMU not an integer");
static_assert((SIMULATION_PERIOD % ANALOG_BARO_PERIOD) == 0,
              "N_DATA_ANALOG_BARO not an integer");
static_assert((SIMULATION_PERIOD % DIGITAL_BARO_PERIOD) == 0,
              "N_DATA_DIGITAL_BARO not an integer");
static_assert((SIMULATION_PERIOD % BARO_CHAMBER_PERIOD) == 0,
              "N_DATA_BARO_CHAMBER not an integer");
static_assert((SIMULATION_PERIOD % TEMP_PERIOD) == 0,
              "N_DATA_TEMP not an integer");
static_assert((SIMULATION_PERIOD % GPS_PERIOD) == 0,
              "N_DATA_GPS not an integer");

/** Number of samples per sensor at each simulator iteration */
constexpr int N_DATA_ACCEL        = SIMULATION_PERIOD / ACCEL_PERIOD;
constexpr int N_DATA_GYRO         = SIMULATION_PERIOD / GYRO_PERIOD;
constexpr int N_DATA_MAGNETO      = SIMULATION_PERIOD / MAGN_PERIOD;
constexpr int N_DATA_IMU          = SIMULATION_PERIOD / IMU_PERIOD;
constexpr int N_DATA_ANALOG_BARO  = SIMULATION_PERIOD / ANALOG_BARO_PERIOD;
constexpr int N_DATA_DIGITAL_BARO = SIMULATION_PERIOD / DIGITAL_BARO_PERIOD;
constexpr int N_DATA_BARO_CHAMBER = SIMULATION_PERIOD / BARO_CHAMBER_PERIOD;
constexpr int N_DATA_GPS          = SIMULATION_PERIOD / GPS_PERIOD;
constexpr int N_DATA_TEMP         = SIMULATION_PERIOD / TEMP_PERIOD;

// Sensors Data
using PayloadAccelerometerSimulatorData =
    Boardcore::AccelerometerSimulatorData<N_DATA_ACCEL>;
using PayloadGyroscopeSimulatorData =
    Boardcore::GyroscopeSimulatorData<N_DATA_GYRO>;
using PayloadMagnetometerSimulatorData =
    Boardcore::MagnetometerSimulatorData<N_DATA_MAGNETO>;
using PayloadGPSSimulatorData = Boardcore::GPSSimulatorData<N_DATA_GPS>;
using PayloadDigitalBarometerSimulatorData =
    Boardcore::BarometerSimulatorData<N_DATA_DIGITAL_BARO>;
using PayloadAnalogBarometerSimulatorData =
    Boardcore::BarometerSimulatorData<N_DATA_ANALOG_BARO>;
using PayloadChamberPressureSimulatorData =
    Boardcore::BarometerSimulatorData<N_DATA_BARO_CHAMBER>;
using PayloadTemperatureSimulatorData =
    Boardcore::TemperatureSimulatorData<N_DATA_TEMP>;

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
                Payload::NASControllerStatus adaStatus)
        : n(adaState.n), e(adaState.e), d(adaState.d), vn(adaState.vn),
          ve(adaState.ve), vd(adaState.vd), qx(adaState.qx), qy(adaState.qy),
          qz(adaState.qz), qw(adaState.qw),
          updating(adaStatus.state == Payload::NASControllerState::ACTIVE)
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

struct WESDataHIL
{
    float windX;
    float windY;

    WESDataHIL(Eigen::Vector2f wind) : windX(wind[0]), windY(wind[1]) {}

    WESDataHIL() : windX(0.0f), windY(0.0f) {}

    void print() { printf("wind: [%f,%f]\n", windX, windY); }
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
};

/**
 * @brief Data strudcture expected by the simulator
 */
struct ActuatorData
{
    NASStateHIL nasState;
    ActuatorsStateHIL actuatorsState;
    WESDataHIL wesData;
    GuidanceDataHIL guidanceData;
    FlagsHIL flags;

    ActuatorData()
        : nasState(), actuatorsState(), wesData(), guidanceData(), flags()
    {
    }

    ActuatorData(NASStateHIL nasState, ActuatorsStateHIL actuatorsState,
                 WESDataHIL wesData, GuidanceDataHIL guidanceData,
                 Payload::FlightModeManager* fmm)
        : nasState(nasState), actuatorsState(actuatorsState), wesData(wesData),
          guidanceData(guidanceData)
    {
        flags.flag_flight =
            (fmm->testState(&Payload::FlightModeManager::state_ascending) ||
                     fmm->testState(
                         &Payload::FlightModeManager::state_drogue_descent) ||
                     fmm->testState(
                         &Payload::FlightModeManager::state_wing_descent)
                 ? 1
                 : 0);
        flags.flag_ascent =
            (fmm->testState(&Payload::FlightModeManager::state_ascending) ? 1
                                                                          : 0);
        flags.flag_burning   = 0;  // Hardcoded to 0 since it's the payload
        flags.flag_airbrakes = 0;  // Hardcoded to 0 since it's the payload
        flags.flag_para1 =
            (fmm->testState(&Payload::FlightModeManager::state_drogue_descent)
                 ? 1
                 : 0);
        flags.flag_para2 =
            (fmm->testState(&Payload::FlightModeManager::state_wing_descent)
                 ? 1
                 : 0);
    }

    void print()
    {
        nasState.print();
        actuatorsState.print();
        wesData.print();
        guidanceData.print();
        flags.print();
    }
};

enum PayloadFlightPhases
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

using PayloadHILTransceiver =
    Boardcore::HILTransceiver<PayloadFlightPhases, SimulatorData, ActuatorData>;
using PayloadHIL =
    Boardcore::HIL<PayloadFlightPhases, SimulatorData, ActuatorData>;

class PayloadHILPhasesManager
    : public Boardcore::HILPhasesManager<PayloadFlightPhases, SimulatorData,
                                         ActuatorData>
{
public:
    explicit PayloadHILPhasesManager(
        std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition)
        : HILPhasesManager<PayloadFlightPhases, SimulatorData, ActuatorData>(
              getCurrentPosition)
    {
        flagsFlightPhases = {{PayloadFlightPhases::SIM_FLYING, false},
                             {PayloadFlightPhases::SIM_ASCENT, false},
                             {PayloadFlightPhases::SIM_BURNING, false},
                             {PayloadFlightPhases::SIM_AEROBRAKES, false},
                             {PayloadFlightPhases::SIM_PARA1, false},
                             {PayloadFlightPhases::SIM_PARA2, false},
                             {PayloadFlightPhases::SIMULATION_STARTED, false},
                             {PayloadFlightPhases::CALIBRATION, false},
                             {PayloadFlightPhases::CALIBRATION_OK, false},
                             {PayloadFlightPhases::ARMED, false},
                             {PayloadFlightPhases::LIFTOFF_PIN_DETACHED, false},
                             {PayloadFlightPhases::LIFTOFF, false},
                             {PayloadFlightPhases::AEROBRAKES, false},
                             {PayloadFlightPhases::APOGEE, false},
                             {PayloadFlightPhases::PARA1, false},
                             {PayloadFlightPhases::PARA2, false},
                             {PayloadFlightPhases::SIMULATION_STOPPED, false}};

        prev_flagsFlightPhases = flagsFlightPhases;

        auto& eventBroker = Boardcore::EventBroker::getInstance();
        eventBroker.subscribe(this, Common::TOPIC_ABK);
        eventBroker.subscribe(this, Common::TOPIC_ADA);
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
        std::vector<PayloadFlightPhases> changed_flags;

        // set true when the first packet from the simulator arrives
        if (isSetTrue(PayloadFlightPhases::SIMULATION_STARTED))
        {
            t_start = Boardcore::TimestampTimer::getTimestamp();

            TRACE("[HIL] ------- SIMULATION STARTED ! ------- \n");
            changed_flags.push_back(PayloadFlightPhases::SIMULATION_STARTED);
        }

        if (flagsFlightPhases[PayloadFlightPhases::SIM_FLYING])
        {
            if (isSetTrue(PayloadFlightPhases::SIM_FLYING))
            {
                registerOutcomes(PayloadFlightPhases::SIM_FLYING);
                TRACE("[HIL] ------- SIMULATOR LIFTOFF ! ------- \n");
                changed_flags.push_back(PayloadFlightPhases::SIM_FLYING);
            }
        }

        /* calling the callbacks subscribed to the changed flags */
        for (unsigned int i = 0; i < changed_flags.size(); i++)
        {
            std::vector<PhasesCallback> callbacksToCall =
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
        outcomes[PayloadFlightPhases::SIM_BURNING].print(t_liftoff);

        printf("Airbrakes exit shadowmode: \n");
        outcomes[PayloadFlightPhases::AEROBRAKES].print(t_liftoff);

        printf("Apogee: \n");
        outcomes[PayloadFlightPhases::APOGEE].print(t_liftoff);

        printf("Parachute 1: \n");
        outcomes[PayloadFlightPhases::PARA1].print(t_liftoff);

        printf("Parachute 2: \n");
        outcomes[PayloadFlightPhases::PARA2].print(t_liftoff);

        printf("Simulation Stopped: \n");
        outcomes[PayloadFlightPhases::SIMULATION_STOPPED].print(t_liftoff);

        // auto cpuMeter = Boardcore::CpuMeter::getCpuStats();
        // printf("max cpu usage: %+.3f\n", cpuMeter.maxValue);
        // printf("avg cpu usage: %+.3f\n", cpuMeter.mean);
        // printf("min free heap: %+.3lu\n", cpuMeter.minFreeHeap);
        // printf("max free stack:%+.3lu\n", cpuMeter.minFreeStack);
    }

private:
    void handleEvent(const Boardcore::Event& e) override
    {
        std::vector<PayloadFlightPhases> changed_flags;
        switch (e)
        {
            case Common::Events::FMM_INIT_ERROR:
                printf("[HIL] ------- INIT FAILED ! ------- \n");
            case Common::Events::FMM_INIT_OK:
                setFlagFlightPhase(PayloadFlightPhases::CALIBRATION, true);
                TRACE("[HIL] ------- CALIBRATION ! ------- \n");
                changed_flags.push_back(PayloadFlightPhases::CALIBRATION);
                break;
            case Common::Events::FLIGHT_DISARMED:
                setFlagFlightPhase(PayloadFlightPhases::CALIBRATION_OK, true);
                TRACE("[HIL] CALIBRATION OK!\n");
                changed_flags.push_back(PayloadFlightPhases::CALIBRATION_OK);
                break;
            case Common::Events::FLIGHT_ARMED:
                setFlagFlightPhase(PayloadFlightPhases::ARMED, true);
                printf("[HIL] ------- READY TO LAUNCH ! ------- \n");
                changed_flags.push_back(PayloadFlightPhases::ARMED);
                break;
            case Common::Events::FLIGHT_LAUNCH_PIN_DETACHED:
                setFlagFlightPhase(PayloadFlightPhases::LIFTOFF_PIN_DETACHED,
                                   true);
                TRACE("[HIL] ------- LIFTOFF PIN DETACHED ! ------- \n");
                changed_flags.push_back(
                    PayloadFlightPhases::LIFTOFF_PIN_DETACHED);
                break;
            case Common::Events::FLIGHT_LIFTOFF:
            case Common::Events::TMTC_FORCE_LAUNCH:
                t_liftoff = Boardcore::TimestampTimer::getTimestamp();
                printf("[HIL] ------- LIFTOFF -------: %f, %f \n",
                       getCurrentPosition().z, getCurrentPosition().vz);
                changed_flags.push_back(PayloadFlightPhases::LIFTOFF);
                break;
            case Common::Events::FLIGHT_APOGEE_DETECTED:
            case Common::Events::CAN_APOGEE_DETECTED:
            case Common::Events::TMTC_FORCE_APOGEE:
                setFlagFlightPhase(PayloadFlightPhases::AEROBRAKES, false);
                registerOutcomes(PayloadFlightPhases::APOGEE);
                printf("[HIL] ------- APOGEE DETECTED ! ------- %f, %f \n",
                       getCurrentPosition().z, getCurrentPosition().vz);
                changed_flags.push_back(PayloadFlightPhases::APOGEE);
                break;
            case Common::Events::FLIGHT_DROGUE_DESCENT:
            case Common::Events::TMTC_FORCE_EXPULSION:
                setFlagFlightPhase(PayloadFlightPhases::PARA1, true);
                registerOutcomes(PayloadFlightPhases::PARA1);
                printf("[HIL] ------- PARA1 ! -------%f, %f \n",
                       getCurrentPosition().z, getCurrentPosition().vz);
                changed_flags.push_back(PayloadFlightPhases::PARA1);
                break;
            case Common::Events::FLIGHT_WING_DESCENT:
            case Common::Events::FLIGHT_DPL_ALT_DETECTED:
            case Common::Events::TMTC_FORCE_DEPLOYMENT:
                setFlagFlightPhase(PayloadFlightPhases::PARA1, false);
                setFlagFlightPhase(PayloadFlightPhases::PARA2, true);
                registerOutcomes(PayloadFlightPhases::PARA2);
                printf("[HIL] ------- PARA2 ! ------- %f, %f \n",
                       getCurrentPosition().z, getCurrentPosition().vz);
                changed_flags.push_back(PayloadFlightPhases::PARA2);
                break;
            case Common::Events::FLIGHT_LANDING_DETECTED:
            case Common::Events::TMTC_FORCE_LANDING:
                t_stop = Boardcore::TimestampTimer::getTimestamp();
                setFlagFlightPhase(PayloadFlightPhases::PARA2, false);
                setFlagFlightPhase(PayloadFlightPhases::SIMULATION_STOPPED,
                                   true);
                changed_flags.push_back(
                    PayloadFlightPhases::SIMULATION_STOPPED);
                registerOutcomes(PayloadFlightPhases::SIMULATION_STOPPED);
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
            std::vector<PhasesCallback> callbacksToCall =
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