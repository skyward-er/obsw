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

#include "HIL.h"

#include <Engine/Actuators/Actuators.h>
#include <Engine/Buses.h>
#include <Engine/Configs/HILSimulationConfig.h>
#include <common/Events.h>
#include <events/EventBroker.h>
#include <hil/HIL.h>

#include "HILData.h"

namespace Engine
{

EngineHILPhasesManager::EngineHILPhasesManager(
    std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition)
    : Boardcore::HILPhasesManager<EngineFlightPhases, SimulatorData,
                                  ActuatorData>(getCurrentPosition)
{
    flagsFlightPhases = {{EngineFlightPhases::SIMULATION_STARTED, false}};

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
    eventBroker.subscribe(this, Common::TOPIC_ENGINE);
    eventBroker.subscribe(this, Common::TOPIC_TARS);
    eventBroker.subscribe(this, Common::TOPIC_ALT);
}

void EngineHILPhasesManager::processFlagsImpl(
    const SimulatorData& simulatorData,
    std::vector<EngineFlightPhases>& changed_flags)
{
    if (simulatorData.signal ==
        static_cast<float>(HILSignal::SIMULATION_STARTED))
    {
        miosix::reboot();
    }

    if (simulatorData.signal ==
        static_cast<float>(HILSignal::SIMULATION_STOPPED))
    {
        Boardcore::EventBroker::getInstance().post(Common::TMTC_FORCE_LANDING,
                                                   Common::TOPIC_TMTC);
    }

    // set true when the first packet from the simulator arrives
    if (isSetTrue(EngineFlightPhases::SIMULATION_STARTED))
    {
        t_start = Boardcore::TimestampTimer::getTimestamp();

        printf("[HIL] ------- SIMULATION STARTED ! ------- \n");
        changed_flags.push_back(EngineFlightPhases::SIMULATION_STARTED);
    }
}

void EngineHILPhasesManager::printOutcomes()
{
    printf("OUTCOMES: (times dt from liftoff)\n\n");
    printf("Simulation time: %.3f [sec]\n\n",
           (double)(t_stop - t_start) / 1000000.0f);
}

void EngineHILPhasesManager::handleEventImpl(
    const Boardcore::Event& e, std::vector<EngineFlightPhases>& changed_flags)
{
    switch (e)
    {
        default:
            printf("%s event\n", Common::getEventString(e).c_str());
    }
}

EngineHIL::EngineHIL()
    : Boardcore::HIL<EngineFlightPhases, SimulatorData, ActuatorData>(
          nullptr, nullptr, [this]() { return updateActuatorData(); },
          1000 / Config::HIL::SIMULATION_RATE.value())
{
}

bool EngineHIL::start()
{
    auto& hilUsart = getModule<Buses>()->getHILUart();

    hilPhasesManager = new EngineHILPhasesManager(
        [&]()
        {
            Boardcore::TimedTrajectoryPoint timedTrajectoryPoint;
            timedTrajectoryPoint.timestamp = Boardcore::Kernel::getOldTick();
            return timedTrajectoryPoint;
        });

    hilTransceiver = new EngineHILTransceiver(hilUsart, hilPhasesManager);

    return Boardcore::HIL<EngineFlightPhases, SimulatorData,
                          ActuatorData>::start();
}

ActuatorData EngineHIL::updateActuatorData()
{
    auto actuators = getModule<Actuators>();

    ActuatorsStateHIL actuatorsStateHIL{
        (actuators->getServoPosition(MAIN_VALVE)),
        (actuators->getServoPosition(OX_VENTING_VALVE)),
        (actuators->getServoPosition(NITROGEN_VALVE)),
        (actuators->getServoPosition(N2_QUENCHING_VALVE)),
    };

    // Returning the feedback for the simulator
    return ActuatorData{actuatorsStateHIL};
}

}  // namespace Engine
