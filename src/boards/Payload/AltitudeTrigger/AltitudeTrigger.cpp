/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro, Federico Mandelli
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

#include "AltitudeTrigger.h"

#include <Payload/BoardScheduler.h>
#include <Payload/Configs/WingConfig.h>
#include <Payload/StateMachines/NASController/NASController.h>
#include <common/Events.h>
#include <events/EventBroker.h>

using namespace Boardcore;
using namespace Common;

namespace Payload
{

AltitudeTrigger::AltitudeTrigger(TaskScheduler *sched)
    : scheduler(sched), running(false), confidence(0),
      deploymentAltitude(WingConfig::ALTITUDE_TRIGGER_DEPLOYMENT_ALTITUDE)
{
}

bool AltitudeTrigger::start()
{
    return scheduler->addTask(std::bind(&AltitudeTrigger::update, this),
                              WingConfig::ALTITUDE_TRIGGER_PERIOD);
}

void AltitudeTrigger::enable()
{
    miosix::Lock<miosix::FastMutex> l(mutex);
    confidence = 0;
    running    = true;
}

void AltitudeTrigger::setDeploymentAltitude(float altitude)
{
    miosix::Lock<miosix::FastMutex> l(mutex);
    deploymentAltitude = altitude;
}

void AltitudeTrigger::disable()
{
    miosix::Lock<miosix::FastMutex> l(mutex);
    running = false;
}

bool AltitudeTrigger::isActive()
{
    miosix::Lock<miosix::FastMutex> l(mutex);
    return running;
}

void AltitudeTrigger::update()
{
    miosix::Lock<miosix::FastMutex> l(mutex);
    if (running)
    {
        // We multiply by -1 to have a positive height
        float height =
            -ModuleManager::getInstance().get<NASController>()->getNasState().d;

        if (height < WingConfig::ALTITUDE_TRIGGER_DEPLOYMENT_ALTITUDE)
        {
            confidence++;
        }
        else
        {
            confidence = 0;
        }
        if (confidence >= WingConfig ::ALTITUDE_TRIGGER_CONFIDENCE)
        {
            confidence = 0;
            EventBroker::getInstance().post(FLIGHT_DPL_ALT_DETECTED,
                                            TOPIC_FLIGHT);
            running = false;
        }
    }
}
}  // namespace Payload
