/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <Main/AltitudeTrigger/AltitudeTrigger.h>
#include <Main/Configs/AltitudeTriggerConfig.h>
#include <Main/StateMachines/ADAController/ADAController.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <events/EventBroker.h>

using namespace Boardcore;
using namespace Common;

namespace Main
{
AltitudeTrigger::AltitudeTrigger(TaskScheduler* sched) : scheduler(sched)
{
    altitude = AltitudeTriggerConfig::ALTITUDE_REFERENCE;
}

bool AltitudeTrigger::start()
{
    size_t result = scheduler->addTask(
        [&]() { update(); }, AltitudeTriggerConfig::ALTITUDE_CHECKER_PERIOD,
        TaskScheduler::Policy::RECOVER);

    return result != 0;
}

void AltitudeTrigger::setDeploymentAltitude(float altitude)
{
    this->altitude = altitude;
}

void AltitudeTrigger::update()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Check the FMM state
    FlightModeManagerStatus status =
        modules.get<FlightModeManager>()->getStatus();

    if (status.state == FlightModeManagerState::DROGUE_DESCENT)
    {
        // Get the estimated ADA altitude
        ADAState state = modules.get<ADAController>()->getADAState();

        // Update the confidence
        if (state.aglAltitude < altitude)
        {
            confidence++;
        }
        else
        {
            confidence = 0;
        }

        if (confidence >= AltitudeTriggerConfig::ALTITUDE_CONFIDENCE_THRESHOLD)
        {
            // We reached the altitude
            confidence = 0;
            EventBroker::getInstance().post(ALTITUDE_TRIGGER_ALTITUDE_REACHED,
                                            TOPIC_FLIGHT);
        }
    }
}
}  // namespace Main