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

#include <Main/Actuators/Actuators.h>
#include <Main/Buses.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <drivers/usart/USART.h>
#include <events/EventBroker.h>
#include <hil/HIL.h>
#include <math.h>
#include <sensors/HILSimulatorData.h>
#include <sensors/SensorInfo.h>
#include <units/Frequency.h>
#include <units/Units.h>
#include <utils/Debug.h>
#include <utils/DependencyManager/DependencyManager.h>
#include <utils/Stats/Stats.h>

#include <chrono>
#include <list>

#include "SensorsConfig.h"
#include "drivers/usart/USART.h"

// FMM
// #include <Main/StateMachines/FlightModeManager/FlightModeManager.h>

// ADA
#include <Main/StateMachines/ADAController/ADAController.h>
#include <Main/StateMachines/ADAController/ADAControllerData.h>
#include <algorithms/ADA/ADAData.h>

// NAS
#include <Main/StateMachines/NASController/NASController.h>
#include <Main/StateMachines/NASController/NASControllerData.h>
#include <algorithms/NAS/NASState.h>

// // ABK
// #include <Main/StateMachines/ABKController/ABKController.h>
// #include <Main/StateMachines/ABKController/ABKControllerData.h>
// #include <algorithms/AirBrakes/AirBrakesInterp.h>

// MEA
#include <Main/StateMachines/MEAController/MEAController.h>
#include <Main/StateMachines/MEAController/MEAControllerData.h>
#include <algorithms/MEA/MEAData.h>

// clang-format off
// Indent to avoid the linter complaining about using namespace
  using namespace Boardcore::Units::Frequency;
  using namespace std::chrono_literals;
// clang-format on

namespace HILConfig
{

constexpr bool IS_FULL_HIL = false;
constexpr bool ENABLE_HW   = true;

/** Period of simulation [ms] */
constexpr auto SIMULATION_RATE    = 10_hz;
constexpr int SIMULATION_RATE_INT = static_cast<int>(SIMULATION_RATE.value());

constexpr auto ACCEL_RATE = Main::Config::Sensors::LSM6DSRX::RATE;
constexpr auto GYRO_RATE  = Main::Config::Sensors::LSM6DSRX::RATE;
constexpr auto MAGN_RATE  = Main::Config::Sensors::LIS2MDL::RATE;
// constexpr auto IMU_RATE          = Main::Config::Sensors::IMU::RATE;
constexpr auto ANALOG_BARO_RATE  = Main::Config::Sensors::ADS131M08::RATE;
constexpr auto DIGITAL_BARO_RATE = Main::Config::Sensors::LPS22DF::RATE;
constexpr auto TEMP_RATE         = SIMULATION_RATE;  // One sample
constexpr auto GPS_RATE          = Main::Config::Sensors::UBXGPS::RATE;
constexpr auto BARO_CHAMBER_RATE = 50_hz;
constexpr auto PITOT_RATE        = 50_hz;

static_assert((static_cast<int>(ACCEL_RATE.value()) % SIMULATION_RATE_INT) == 0,
              "N_DATA_ACCEL not an integer");
static_assert((static_cast<int>(GYRO_RATE.value()) % SIMULATION_RATE_INT) == 0,
              "N_DATA_GYRO not an integer");
static_assert((static_cast<int>(MAGN_RATE.value()) % SIMULATION_RATE_INT) == 0,
              "N_DATA_MAGN not an integer");
// static_assert((static_cast<int>(IMU_RATE.value()) % SIMULATION_RATE_INT) ==
// 0,
//               "N_DATA_IMU not an integer");
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
static_assert((static_cast<int>(BARO_CHAMBER_RATE.value()) %
               SIMULATION_RATE_INT) == 0,
              "N_DATA_BARO_CHAMBER not an integer");
static_assert((static_cast<int>(PITOT_RATE.value()) % SIMULATION_RATE_INT) == 0,
              "N_DATA_PITOT not an integer");

/** Number of samples per sensor at each simulator iteration */
constexpr int N_DATA_ACCEL   = ACCEL_RATE.value() / SIMULATION_RATE_INT;
constexpr int N_DATA_GYRO    = GYRO_RATE.value() / SIMULATION_RATE_INT;
constexpr int N_DATA_MAGNETO = MAGN_RATE.value() / SIMULATION_RATE_INT;
// constexpr int N_DATA_IMU     = IMU_RATE.value() / SIMULATION_RATE_INT;
constexpr int N_DATA_ANALOG_BARO =
    ANALOG_BARO_RATE.value() / SIMULATION_RATE_INT;
constexpr int N_DATA_DIGITAL_BARO =
    DIGITAL_BARO_RATE.value() / SIMULATION_RATE_INT;
constexpr int N_DATA_GPS  = TEMP_RATE.value() / SIMULATION_RATE_INT;
constexpr int N_DATA_TEMP = GPS_RATE.value() / SIMULATION_RATE_INT;
constexpr int N_DATA_BARO_CHAMBER =
    BARO_CHAMBER_RATE.value() / SIMULATION_RATE_INT;
constexpr int N_DATA_PITOT = PITOT_RATE.value() / SIMULATION_RATE_INT;

// Sensors Data
using MainAccelerometerSimulatorData =
    Boardcore::AccelerometerSimulatorData<N_DATA_ACCEL>;
using MainGyroscopeSimulatorData =
    Boardcore::GyroscopeSimulatorData<N_DATA_GYRO>;
using MainMagnetometerSimulatorData =
    Boardcore::MagnetometerSimulatorData<N_DATA_MAGNETO>;
using MainGPSSimulatorData = Boardcore::GPSSimulatorData<N_DATA_GPS>;
using MainBarometerSimulatorData =
    Boardcore::BarometerSimulatorData<N_DATA_DIGITAL_BARO>;
using MainChamberPressureSimulatorData =
    Boardcore::BarometerSimulatorData<N_DATA_BARO_CHAMBER>;
using MainPitotSimulatorData = Boardcore::PitotSimulatorData<N_DATA_PITOT>;
using MainTemperatureSimulatorData =
    Boardcore::TemperatureSimulatorData<N_DATA_TEMP>;

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

    ADAStateHIL(Boardcore::ADAState adaState)
        : mslAltitude(adaState.mslAltitude), aglAltitude(adaState.aglAltitude),
          verticalSpeed(adaState.verticalSpeed), apogeeDetected(0), updating(0)
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

    NASStateHIL(Boardcore::NASState nasState)
        : n(nasState.n), e(nasState.e), d(nasState.d), vn(nasState.vn),
          ve(nasState.ve), vd(nasState.vd), qx(nasState.qx), qy(nasState.qy),
          qz(nasState.qz), qw(nasState.qw), updating(0)
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

    // AirBrakesStateHIL(Main::ABKControllerStatus abkStatus)
    //     : updating(abkStatus.state == Main::ABKControllerState::ACTIVE)
    // {
    // }

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

    MEAStateHIL(Boardcore::MEAState state)
        : correctedPressure(state.estimatedPressure),
          estimatedMass(state.estimatedMass),
          estimatedApogee(state.estimatedApogee), updating(true)
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

    ActuatorData(ADAStateHIL adaState, NASStateHIL nasState,
                 AirBrakesStateHIL airBrakesState, MEAStateHIL meaState,
                 ActuatorsStateHIL actuatorsState)
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
    SHUTDOWN,
    AEROBRAKES,
    APOGEE,
    PARA1,
    PARA2,
    SIMULATION_STOPPED
};

class MainHILTransceiver
    : public Boardcore::HILTransceiver<MainFlightPhases, SimulatorData,
                                       ActuatorData>
{
    using Boardcore::HILTransceiver<MainFlightPhases, SimulatorData,
                                    ActuatorData>::HILTransceiver;
};

class MainHILPhasesManager
    : public Boardcore::HILPhasesManager<MainFlightPhases, SimulatorData,
                                         ActuatorData>
{
public:
    explicit MainHILPhasesManager(
        std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition)
        : Boardcore::HILPhasesManager<MainFlightPhases, SimulatorData,
                                      ActuatorData>(getCurrentPosition)
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

    void processFlagsImpl(const SimulatorData& simulatorData,
                          std::vector<MainFlightPhases>& changed_flags) override
    {
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
        if (isSetTrue(MainFlightPhases::SIMULATION_STARTED))
        {
            t_start = Boardcore::TimestampTimer::getTimestamp();

            printf("[HIL] ------- SIMULATION STARTED ! ------- \n");
            changed_flags.push_back(MainFlightPhases::SIMULATION_STARTED);
        }

        if (flagsFlightPhases[MainFlightPhases::SIM_FLYING])
        {
            if (isSetTrue(MainFlightPhases::SIM_FLYING))
            {
                registerOutcomes(MainFlightPhases::SIM_FLYING);
                printf("[HIL] ------- SIMULATOR LIFTOFF ! ------- \n");
                changed_flags.push_back(MainFlightPhases::SIM_FLYING);
            }
        }
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
    void handleEventImpl(const Boardcore::Event& e,
                         std::vector<MainFlightPhases>& changed_flags) override
    {
        switch (e)
        {
            case Common::Events::FMM_INIT_ERROR:
                printf("[HIL] ------- INIT FAILED ! ------- \n");
            case Common::Events::FMM_INIT_OK:
                setFlagFlightPhase(MainFlightPhases::CALIBRATION, true);
                printf("[HIL] ------- CALIBRATION ! ------- \n");
                changed_flags.push_back(MainFlightPhases::CALIBRATION);
                break;
            case Common::Events::FLIGHT_DISARMED:
                setFlagFlightPhase(MainFlightPhases::CALIBRATION_OK, true);
                printf("[HIL] CALIBRATION OK!\n");
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
                printf("[HIL] ------- LIFTOFF PIN DETACHED ! ------- \n");
                changed_flags.push_back(MainFlightPhases::LIFTOFF_PIN_DETACHED);
                break;
            case Common::Events::FLIGHT_LIFTOFF:
            case Common::Events::TMTC_FORCE_LAUNCH:
                t_liftoff = Boardcore::TimestampTimer::getTimestamp();
                printf("[HIL] ------- LIFTOFF -------: %f, %f \n",
                       getCurrentPosition().z, getCurrentPosition().vz);
                changed_flags.push_back(MainFlightPhases::LIFTOFF);
                break;
            case Common::Events::FLIGHT_MOTOR_SHUTDOWN:
                printf("[HIL] ------- SHUTDOWN -------: %f, %f \n",
                       getCurrentPosition().z, getCurrentPosition().vz);
                changed_flags.push_back(MainFlightPhases::SHUTDOWN);
            case Common::Events::ABK_SHADOW_MODE_TIMEOUT:
                setFlagFlightPhase(MainFlightPhases::AEROBRAKES, true);
                registerOutcomes(MainFlightPhases::AEROBRAKES);
                printf("[HIL] ABK shadow mode timeout\n");
                changed_flags.push_back(MainFlightPhases::AEROBRAKES);
                break;
            case Common::Events::ADA_SHADOW_MODE_TIMEOUT:
                printf("[HIL] ADA shadow mode timeout\n");
                break;
            case Common::Events::ABK_DISABLE:
                setFlagFlightPhase(MainFlightPhases::AEROBRAKES, false);
                printf("[HIL] ABK disabled\n");
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
                printf("[HIL] ------- SIMULATION STOPPED ! -------: %f \n\n\n",
                       (double)t_stop / 1000000.0f);
                printOutcomes();
                break;
            default:
                printf("%s event\n", Common::getEventString(e).c_str());
        }
    }
};

class MainHIL
    : public Boardcore::HIL<MainFlightPhases, SimulatorData, ActuatorData>,
      public Boardcore::InjectableWithDeps<
          Main::Buses, Main::Actuators, Main::FlightModeManager,
          Main::ADAController, Main::NASController,
          Main::MEAController /*, Main::ABKController */>

{
public:
    MainHIL()
        : Boardcore::HIL<MainFlightPhases, SimulatorData, ActuatorData>(
              nullptr, nullptr, [this]() { return updateActuatorData(); },
              1000 / SIMULATION_RATE_INT)
    {
    }

    bool start() override
    {
        auto* nas      = getModule<Main::NASController>();
        auto& hilUsart = getModule<Main::Buses>()->getHILUart();

        hilPhasesManager = new MainHILPhasesManager(
            [nas]()
            { return Boardcore::TimedTrajectoryPoint(nas->getNASState()); });

        hilTransceiver = new MainHILTransceiver(hilUsart, hilPhasesManager);

        return Boardcore::HIL<MainFlightPhases, SimulatorData,
                              ActuatorData>::start();
    }

private:
    ActuatorData updateActuatorData()
    {
        auto actuators = getModule<Main::Actuators>();

        ADAStateHIL adaStateHIL{
            getModule<Main::ADAController>()->getADAState()};

        NASStateHIL nasStateHIL{
            getModule<Main::NASController>()->getNASState()};

        AirBrakesStateHIL abkStateHIL{
            /* modules.get<ABKController>()->getStatus() */};

        MEAStateHIL meaStateHIL{
            getModule<Main::MEAController>()->getMEAState()};

        ActuatorsStateHIL actuatorsStateHIL{
            actuators->getServoPosition(ServosList::AIR_BRAKES_SERVO),
            actuators->getServoPosition(ServosList::EXPULSION_SERVO),
            (actuators->isCanServoOpen(ServosList::MAIN_VALVE) ? 1.f : 0.f),
            (actuators->isCanServoOpen(ServosList::VENTING_VALVE) ? 1.f : 0.f),
            static_cast<float>(miosix::gpios::mainDeploy::value())};

        // Returning the feedback for the simulator
        return ActuatorData{adaStateHIL, nasStateHIL, abkStateHIL, meaStateHIL,
                            actuatorsStateHIL};
    };
};

}  // namespace HILConfig