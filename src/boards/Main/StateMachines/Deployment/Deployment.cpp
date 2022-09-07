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

#include <Main/Actuators/Actuators.h>
#include <Main/Configs/ActuatorsConfigs.h>
#include <Main/Configs/DeploymentConfig.h>
#include <common/events/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <logger/Logger.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;
using namespace Main::DeploymentConfig;
using namespace Main::ActuatorsConfigs;
using namespace Common;

namespace Main
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

            Actuators::getInstance().setServoAngle(EXPULSION_SERVO,
                                                   DPL_SERVO_EJECT_POS);
            Actuators::getInstance().enableServo(EXPULSION_SERVO);

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
        case DPL_WIGGLE:
        {
            return wiggleServo();
        }
        case DPL_NC_OPEN:
        {
            Actuators::getInstance().setServo(EXPULSION_SERVO,
                                              DPL_SERVO_EJECT_POS);
            break;
        }
        case DPL_NC_RESET:
        {
            Actuators::getInstance().setServo(EXPULSION_SERVO, 0);
            break;
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
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

Deployment::~Deployment() { EventBroker::getInstance().unsubscribe(this); }

void Deployment::logStatus(DeploymentState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    Logger::getInstance().log(status);
}

void Deployment::wiggleServo()
{
    for (int i = 0; i < 2; i++)
    {
        Actuators::getInstance().setServoAngle(EXPULSION_SERVO,
                                               DPL_SERVO_EJECT_POS);
        miosix::Thread::sleep(500);
        Actuators::getInstance().setServoAngle(EXPULSION_SERVO,
                                               DPL_SERVO_RESET_POS);
        miosix::Thread::sleep(500);
    }
}

void Deployment::startCutting()
{
    Actuators::getInstance().cutter1.high();
    Actuators::getInstance().ledRed.high();
}

void Deployment::stopCutting()
{
    Actuators::getInstance().cutter1.low();
    Actuators::getInstance().ledRed.low();
}

}  // namespace Main
