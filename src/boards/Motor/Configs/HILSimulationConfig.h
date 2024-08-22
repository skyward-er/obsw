/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Emilio Corigliano, Giulia Facchi
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

#include <Motor/Buses.h>
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

/** sampling periods of sensors [ms] */
constexpr auto BARO_CHAMBER_RATE = Motor::Config::Sensors::ADS131M08::SAMPLING_RATE;

static_assert((static_cast<int>(BARO_CHAMBER_RATE.value()) % SIMULATION_RATE_INT) == 0,
              "N_DATA_BARO_CHAMBER not an integer");


/** Number of samples per sensor at each simulator iteration */
constexpr int N_DATA_BARO_CHAMBER = BARO_CHAMBER_RATE.value() / SIMULATION_RATE_INT;

// Sensors Data
using MotorHILChamberBarometerData =
    Boardcore::BarometerSimulatorData<N_DATA_BARO_CHAMBER>;

struct ActuatorsStateHIL
{
    float mainValvePercentage    = 0;
    float ventingValvePercentage = 0;

    ActuatorsStateHIL()
        : mainValvePercentage(0.0f), ventingValvePercentage(0.0f)
    {
    }

    ActuatorsStateHIL(float mainValvePercentage, float ventingValvePercentage)
        : mainValvePercentage(mainValvePercentage),
          ventingValvePercentage(ventingValvePercentage)
    {
    }

    void print()
    {
        printf(
            "mainValve: %f perc\n"
            "venting: %f perc\n",
            mainValvePercentage * 100, ventingValvePercentage * 100);
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
    MotorHILChamberBarometerData pressureChamber;
    float signal;
};

/**
 * @brief Data structure expected by the simulator
 */
struct ActuatorData
{
    ActuatorsStateHIL actuatorsState;

    ActuatorData() : actuatorsState() {}

    ActuatorData(ActuatorsStateHIL actuatorsState)
        : actuatorsState(actuatorsState)
    {
    }

    void print() { actuatorsState.print(); }
};

enum MotorFlightPhases
{
    SIMULATION_STARTED
};

class MotorHILTransceiver
    : public Boardcore::HILTransceiver<MotorFlightPhases, SimulatorData,
                                       ActuatorData>,
      public Boardcore::Module
{
    using Boardcore::HILTransceiver<MotorFlightPhases, SimulatorData,
                                    ActuatorData>::HILTransceiver;
};

class MainHIL
    : public Boardcore::HIL<MotorFlightPhases, SimulatorData, ActuatorData>,
      public Boardcore::Injectable
{
    using Boardcore::HIL<MotorFlightPhases, SimulatorData, ActuatorData>::HIL;

    virtual ~MotorHIL() = default;
};

class MotorHILPhasesManager
    : public Boardcore::HILPhasesManager<MotorFlightPhases, SimulatorData,
                                         ActuatorData>
{
public:
    explicit MotorHILPhasesManager(
        std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition)
        : Boardcore::HILPhasesManager<MotorFlightPhases, SimulatorData,
                                      ActuatorData>(getCurrentPosition)
    {
        flagsFlightPhases = {{MotorFlightPhases::SIMULATION_STARTED, false}};

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
        updateSimulatorFlags(simulatorData);

        std::vector<MotorFlightPhases> changed_flags;

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
        if (isSetTrue(MotorFlightPhases::SIMULATION_STARTED))
        {
            t_start = Boardcore::TimestampTimer::getTimestamp();

            TRACE("[HIL] ------- SIMULATION STARTED ! ------- \n");
            changed_flags.push_back(MotorFlightPhases::SIMULATION_STARTED);
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
    }

private:
    void handleEvent(const Boardcore::Event& e) override
    {
        std::vector<MotorFlightPhases> changed_flags;
        switch (e)
        {
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

    /**
     * @brief Updates the flags of the object with the flags sent from matlab
     * and checks for the apogee
     */
    void updateSimulatorFlags(const SimulatorData& simulatorData) {}
};

}  // namespace HILConfig
