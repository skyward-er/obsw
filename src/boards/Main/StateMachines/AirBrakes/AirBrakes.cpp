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

#include "AirBrakes.h"

#include <Main/Actuators/Actuators.h>
#include <Main/Configs/ActuatorsConfigs.h>
#include <Main/Configs/AirBrakesConfig.h>
#include <Main/events/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

using namespace Boardcore;
using namespace Main::AirBrakesConfigs;
using namespace Main::ActuatorsConfigs;

namespace Main
{

AirBrakes::AirBrakes() : FSM(&AirBrakes::state_init)
{
    memset(&status, 0, sizeof(AirBrakesStatus));
    EventBroker::getInstance().subscribe(this, TOPIC_ABK);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

AirBrakes::~AirBrakes() { EventBroker::getInstance().unsubscribe(this); }

AirBrakesStatus AirBrakes::getStatus() { return status; }

void AirBrakes::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(INIT);

            Actuators::getInstance().setServoAngle(AIRBRAKES_SERVO,
                                                   DPL_SERVO_RESET_POS);
            Actuators::getInstance().enableServo(AIRBRAKES_SERVO);

            return transition(&AirBrakes::state_idle);
        }
    }
}

void AirBrakes::state_idle(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(IDLE);
        }
        case ABK_WIGGLE:
        {
            return wiggleServo();
        }
        case ABK_OPEN:
        {
            Actuators::getInstance().setServoAngle(AIRBRAKES_SERVO,
                                                   ABK_SERVO_ROTATION);
            break;
        }
        case ABK_RESET:
        {
            Actuators::getInstance().setServoAngle(AIRBRAKES_SERVO, 0);
            break;
        }
        case FLIGHT_LIFTOFF_DETECTED:
        {
            return transition(&AirBrakes::state_shadow_mode);
        }
    }
}

void AirBrakes::state_shadow_mode(const Event& event)
{
    static uint16_t shadowModeTimeoutEventId = -1;

    switch (event)
    {
        case EV_ENTRY:
        {
            shadowModeTimeoutEventId =
                EventBroker::getInstance().postDelayed<SHADOW_MODE_TIMEOUT>(
                    Boardcore::Event{ABK_SHADOW_MODE_TIMEOUT}, TOPIC_ABK);

            return logStatus(SHADOW_MODE);
        }
        case ABK_SHADOW_MODE_TIMEOUT:
        {
            return transition(&AirBrakes::state_active);
        }
        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(shadowModeTimeoutEventId);
        }
    }
}

void AirBrakes::state_active(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(ACTIVE);
        }
        case FLIGHT_APOGEE_DETECTED:
        {
            return transition(&AirBrakes::state_end);
        }
        case ABK_DISABLE:
        {
            return transition(&AirBrakes::state_end);
        }
    }
}

void AirBrakes::state_end(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            Actuators::getInstance().setServoAngle(AIRBRAKES_SERVO, 0);

            return logStatus(END);
        }
    }
}

void AirBrakes::logStatus(AirBrakesState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    Logger::getInstance().log(state);
}

void AirBrakes::wiggleServo()
{
    for (int i = 0; i < 2; i++)
    {
        Actuators::getInstance().setServoAngle(AIRBRAKES_SERVO,
                                               ABK_SERVO_ROTATION);
        miosix::Thread::sleep(500);
        Actuators::getInstance().setServoAngle(AIRBRAKES_SERVO, 0);
        miosix::Thread::sleep(500);
    }
}

}  // namespace Main
