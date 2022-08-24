/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include "Deployment.h"

#include <Payload/Actuators/Actuators.h>
#include <Payload/Configs/ActuatorsConfigs.h>
#include <Payload/Configs/DeploymentConfig.h>
#include <common/events/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <logger/Logger.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;
using namespace Payload::DeploymentConfig;
using namespace Payload::ActuatorsConfigs;
using namespace Common;

namespace Payload
{

DeploymentStatus Deployment::getStatus()
{
    PauseKernelLock lock;
    return status;
}

void Deployment::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(DeploymentState::INIT);

            return transition(&Deployment::state_idle);
        }
    }
}

void Deployment::state_idle(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(DeploymentState::IDLE);
        }
        case DPL_CUT_DROGUE:
        {
            return transition(&Deployment::state_cutting);
        }
    }
}

void Deployment::state_cutting(const Event& event)
{
    static uint16_t ncCuttinTimeoutEventId = -1;

    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(DeploymentState::CUTTING);

            startCutting();

            ncCuttinTimeoutEventId =
                EventBroker::getInstance().postDelayed<CUT_DURATION>(
                    Boardcore::Event{DPL_CUT_TIMEOUT}, TOPIC_DPL);
            break;
        }
        case DPL_CUT_TIMEOUT:
        {
            stopCutting();

            return transition(&Deployment::state_idle);
        }
        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(ncCuttinTimeoutEventId);
            break;
        }
    }
}

Deployment::Deployment() : FSM(&Deployment::state_init)
{
    EventBroker::getInstance().subscribe(this, TOPIC_DPL);
}

Deployment::~Deployment() { EventBroker::getInstance().unsubscribe(this); }

void Deployment::logStatus(DeploymentState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    Logger::getInstance().log(status);
}

void Deployment::startCutting() { Actuators::getInstance().cuttersOn(); }

void Deployment::stopCutting() { Actuators::getInstance().cuttersOff(); }

}  // namespace Payload
