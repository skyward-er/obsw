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

#include <Main/Actuators/Actuators.h>
#include <Main/Configs/DeploymentConfig.h>
#include <Main/StateMachines/Deployment/Deployment.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

using namespace Boardcore;
using namespace Common;

namespace Main
{
Deployment::Deployment() : FSM(&Deployment::state_idle)
{
    EventBroker::getInstance().subscribe(this, TOPIC_DPL);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

DeploymentStatus Deployment::getStatus()
{
    // Need to pause the kernel
    miosix::PauseKernelLock l;
    return status;
}

void Deployment::state_idle(const Event& event)
{
    ModuleManager& modules = ModuleManager::getInstance();
    switch (event)
    {
        case EV_ENTRY:
        {
            // TODO stop cutting with Actuators module

            return logStatus(DeploymentState::IDLE);
        }
        case FLIGHT_DPL_ALT_DETECTED:
        {
            return transition(&Deployment::state_cutting);
        }
    }
}

void Deployment::state_cutting(const Event& event)
{
    static uint16_t ncCuttingTimeoutEventId = 0;

    ModuleManager& modules = ModuleManager::getInstance();

    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(DeploymentState::CUTTING);

            // TODO start cutting with Actuators module

            ncCuttingTimeoutEventId = EventBroker::getInstance().postDelayed(
                DPL_CUT_TIMEOUT, TOPIC_DPL, DeploymentConfig::CUT_DURATION);

            break;
        }
        case EV_EXIT:
        {
            return EventBroker::getInstance().removeDelayed(
                ncCuttingTimeoutEventId);
        }
        case DPL_CUT_TIMEOUT:
        {
            return transition(&Deployment::state_idle);
        }
    }
}

void Deployment::logStatus(DeploymentState state)
{
    {
        miosix::PauseKernelLock lock;
        // Update the current FSM state
        status.timestamp = TimestampTimer::getTimestamp();
        status.state     = state;
    }

    // Log the status
    Logger::getInstance().log(status);
}
}  // namespace Main