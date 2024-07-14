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

#include <Payload/Buses.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Payload/StateMachines/NASController/NASController.h>
#include <Payload/StateMachines/WingController/WingController.h>
#include <Payload/WindEstimationScheme/WindEstimation.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <drivers/usart/USART.h>
#include <events/EventBroker.h>
#include <hil/HIL.h>
#include <math.h>
#include <sensors/HILSimulatorData.h>
#include <sensors/SensorInfo.h>
#include <units/Frequency.h>
#include <utils/Debug.h>
#include <utils/DependencyManager/DependencyManager.h>
#include <utils/Stats/Stats.h>

#include <chrono>
#include <list>
#include <utils/ModuleManager/ModuleManager.hpp>

#include "SensorsConfig.h"

// // NAS
#include <Payload/StateMachines/NASController/NASControllerData.h>
#include <algorithms/NAS/NASState.h>

// // WingController
#include <Payload/StateMachines/WingController/WingControllerData.h>

// clang-format off
// Indent to avoid the linter complaining about using namespace
  using namespace Boardcore::Units::Frequency;
  using namespace std::chrono_literals;
// clang-format on

namespace HILConfig
{

constexpr bool ENABLE_HW = true;

/** Frequency of the simulation */
constexpr auto SIMULATION_RATE    = 10_hz;
constexpr int SIMULATION_RATE_INT = static_cast<int>(SIMULATION_RATE.value());

/** sample frequency of sensor (samples/second) */
constexpr auto ACCEL_RATE = Payload::Config::Sensors::LSM6DSRX::SAMPLING_RATE;
constexpr auto GYRO_RATE  = Payload::Config::Sensors::LSM6DSRX::SAMPLING_RATE;
constexpr auto MAGN_RATE  = Payload::Config::Sensors::LIS2MDL::SAMPLING_RATE;
constexpr auto IMU_RATE   = Payload::Config::Sensors::IMU::SAMPLING_RATE;
constexpr auto ANALOG_BARO_RATE =
    Payload::Config::Sensors::ADS131M08::SAMPLING_RATE;
constexpr auto DIGITAL_BARO_RATE =
    Payload::Config::Sensors::LPS22DF::SAMPLING_RATE;
constexpr auto TEMP_RATE = SIMULATION_RATE;  // One sample
constexpr auto GPS_RATE  = Payload::Config::Sensors::UBXGPS::SAMPLING_RATE;

static_assert((static_cast<int>(ACCEL_RATE.value()) % SIMULATION_RATE_INT) == 0,
              "N_DATA_ACCEL not an integer");
static_assert((static_cast<int>(GYRO_RATE.value()) % SIMULATION_RATE_INT) == 0,
              "N_DATA_GYRO not an integer");
static_assert((static_cast<int>(MAGN_RATE.value()) % SIMULATION_RATE_INT) == 0,
              "N_DATA_MAGN not an integer");
static_assert((static_cast<int>(IMU_RATE.value()) % SIMULATION_RATE_INT) == 0,
              "N_DATA_IMU not an integer");
static_assert((static_cast<int>(ANALOG_BARO_RATE.value()) %
               SIMULATION_RATE_INT) == 0,
              "N_DATA_ANALOG_BARO not an integer");
static_assert((static_cast<int>(DIGITAL_BARO_RATE.value()) %
               SIMULATION_RATE_INT) == 0,
              "N_DATA_DIGITAL_BARO not an integer");
static_assert((static_cast<int>(TEMP_RATE.value()) % SIMULATION_RATE_INT) == 0,
              "N_DATA_TEMP not an integer");
static_assert((static_cast<int>(GPS_RATE.value()) % SIMULATION_RATE_INT) == 0,
              "N_DATA_GPS not an integer");

/** Number of samples per sensor at each simulator iteration */
constexpr int N_DATA_ACCEL   = ACCEL_RATE.value() / SIMULATION_RATE_INT;
constexpr int N_DATA_GYRO    = GYRO_RATE.value() / SIMULATION_RATE_INT;
constexpr int N_DATA_MAGNETO = MAGN_RATE.value() / SIMULATION_RATE_INT;
constexpr int N_DATA_IMU     = IMU_RATE.value() / SIMULATION_RATE_INT;
constexpr int N_DATA_ANALOG_BARO =
    ANALOG_BARO_RATE.value() / SIMULATION_RATE_INT;
constexpr int N_DATA_DIGITAL_BARO =
    DIGITAL_BARO_RATE.value() / SIMULATION_RATE_INT;
constexpr int N_DATA_GPS  = TEMP_RATE.value() / SIMULATION_RATE_INT;
constexpr int N_DATA_TEMP = GPS_RATE.value() / SIMULATION_RATE_INT;

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
using PayloadTemperatureSimulatorData =
    Boardcore::TemperatureSimulatorData<N_DATA_TEMP>;

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
    float signal;
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
    float signal;

    ActuatorData() : nasState(), actuatorsState(), wesData(), guidanceData() {}

    ActuatorData(NASStateHIL nasState, ActuatorsStateHIL actuatorsState,
                 WESDataHIL wesData, GuidanceDataHIL guidanceData, float signal)
        : nasState(nasState), actuatorsState(actuatorsState), wesData(wesData),
          guidanceData(guidanceData), signal(signal)
    {
    }

    void print()
    {
        nasState.print();
        actuatorsState.print();
        wesData.print();
        guidanceData.print();
    }
};

enum PayloadFlightPhases
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

class PayloadHILTransceiver
    : public Boardcore::HILTransceiver<PayloadFlightPhases, SimulatorData,
                                       ActuatorData>
{
    using Boardcore::HILTransceiver<PayloadFlightPhases, SimulatorData,
                                    ActuatorData>::HILTransceiver;
};

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
        flagsFlightPhases = {{PayloadFlightPhases::SIMULATION_STARTED, false},
                             {PayloadFlightPhases::INITIALIZED, false},
                             {PayloadFlightPhases::CALIBRATION, false},
                             {PayloadFlightPhases::CALIBRATION_OK, false},
                             {PayloadFlightPhases::ARMED, false},
                             {PayloadFlightPhases::LIFTOFF_PIN_DETACHED, false},
                             {PayloadFlightPhases::LIFTOFF, false},
                             {PayloadFlightPhases::MOTOR_SHUTDOWN, false},
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

        if (simulatorData.signal == 1)
        {
            miosix::reboot();
        }

        if (simulatorData.signal == 2)
        {
            Boardcore::EventBroker::getInstance().post(
                Common::TMTC_FORCE_LANDING, Common::TOPIC_TMTC);
        }

        if (simulatorData.signal == 3)
        {
            Boardcore::EventBroker::getInstance().post(
                Common::TMTC_FORCE_LAUNCH, Common::TOPIC_TMTC);
        }

        // set true when the first packet from the simulator arrives
        if (isSetTrue(PayloadFlightPhases::SIMULATION_STARTED))
        {
            t_start = Boardcore::TimestampTimer::getTimestamp();

            printf("[HIL] ------- SIMULATION STARTED ! ------- \n");
            changed_flags.push_back(PayloadFlightPhases::SIMULATION_STARTED);
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

        printf("Motor shutdown: \n");
        outcomes[PayloadFlightPhases::MOTOR_SHUTDOWN].print(t_liftoff);

        printf("Apogee: \n");
        outcomes[PayloadFlightPhases::APOGEE].print(t_liftoff);

        printf("Parachute 1: \n");
        outcomes[PayloadFlightPhases::PARA1].print(t_liftoff);

        printf("Parachute 2: \n");
        outcomes[PayloadFlightPhases::PARA2].print(t_liftoff);

        printf("Simulation Stopped: \n");
        outcomes[PayloadFlightPhases::SIMULATION_STOPPED].print(t_liftoff);
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
                setFlagFlightPhase(PayloadFlightPhases::INITIALIZED, true);
                TRACE("[HIL] ------- INITIALIZED ! ------- \n");
                changed_flags.push_back(PayloadFlightPhases::INITIALIZED);
                break;
            case Common::Events::FMM_CALIBRATE:
            case Common::Events::CAN_CALIBRATE:
            case Common::Events::TMTC_CALIBRATE:
                setFlagFlightPhase(PayloadFlightPhases::CALIBRATION, true);
                TRACE("[HIL] ------- CALIBRATION ! ------- \n");
                changed_flags.push_back(PayloadFlightPhases::CALIBRATION);
                break;
            case Common::Events::FLIGHT_DISARMED:
                setFlagFlightPhase(PayloadFlightPhases::CALIBRATION_OK, true);
                TRACE("[HIL] ------- CALIBRATION OK ! ------- \n");
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
            case Common::Events::FLIGHT_MOTOR_SHUTDOWN:
                setFlagFlightPhase(PayloadFlightPhases::MOTOR_SHUTDOWN, true);
                registerOutcomes(PayloadFlightPhases::MOTOR_SHUTDOWN);
                printf("[HIL] ------- MOTOR SHUTDOWN ! ------- %f, %f \n",
                       getCurrentPosition().z, getCurrentPosition().vz);
                changed_flags.push_back(PayloadFlightPhases::MOTOR_SHUTDOWN);
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

class PayloadHIL
    : public Boardcore::HIL<PayloadFlightPhases, SimulatorData, ActuatorData>,
      public Boardcore::InjectableWithDeps<
          Payload::Buses, Payload::Actuators, Payload::FlightModeManager,
          Payload::WindEstimation, Payload::WingController,
          Payload::NASController>
{

public:
    PayloadHIL()
        : Boardcore::HIL<PayloadFlightPhases, SimulatorData, ActuatorData>(
              nullptr, nullptr, [this]() { return updateActuatorData(); },
              1000 / SIMULATION_RATE_INT)
    {
    }

    bool start() override
    {
        auto* nas      = getModule<Payload::NASController>();
        auto& hilUsart = getModule<Payload::Buses>()->HILUart();

        hilPhasesManager = new PayloadHILPhasesManager(
            [nas]()
            { return Boardcore::TimedTrajectoryPoint(nas->getNasState()); });

        hilTransceiver = new PayloadHILTransceiver(hilUsart, hilPhasesManager);

        return Boardcore::HIL<PayloadFlightPhases, SimulatorData,
                              ActuatorData>::start();
    }

private:
    ActuatorData updateActuatorData()
    {
        auto nas = getModule<Payload::NASController>();
        auto wes = getModule<Payload::WindEstimation>();
        auto fmm = getModule<Payload::FlightModeManager>();
        // auto wing      = getModule<Payload::WingController>();
        auto actuators = getModule<Payload::Actuators>();

        NASStateHIL nasStateHIL(nas->getNasState(), nas->getStatus());

        ActuatorsStateHIL actuatorsStateHIL(
            actuators->getServoPosition(ServosList::PARAFOIL_LEFT_SERVO),
            actuators->getServoPosition(ServosList::PARAFOIL_RIGHT_SERVO),
            static_cast<float>(miosix::gpios::mainDeploy::value()));

        WESDataHIL wesDataHIL(wes->getWindEstimationScheme());

        auto deltaA =
            actuators->getServoPosition(ServosList::PARAFOIL_LEFT_SERVO) -
            actuators->getServoPosition(ServosList::PARAFOIL_RIGHT_SERVO);

        Eigen::Vector2f heading;
        // TODO: calculate this
        auto psiRef = 0 /* wing->emGuidance.calculateTargetAngle(
            {nasStateHIL.n, nasStateHIL.e, nasStateHIL.d}, heading) */
            ;

        GuidanceDataHIL guidanceData(psiRef, deltaA, heading[0], heading[1]);

        // Returning the feedback for the simulator
        return ActuatorData(
            nasStateHIL, actuatorsStateHIL, wesDataHIL, guidanceData,
            (fmm->testState(&Payload::FlightModeManager::state_ascending) ? 3
                                                                          : 0));
    };
};

}  // namespace HILConfig