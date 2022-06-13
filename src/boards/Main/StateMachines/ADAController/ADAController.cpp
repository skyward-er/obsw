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

#include "ADAController.h"

#include <Main/Configs/ADAConfig.h>
#include <Main/events/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <miosix.h>

using namespace Boardcore;
using namespace Main::ADAConfig;

namespace Main
{

ADAController::ADAController() : FSM(&ADAController::state_idle)
{
    memset(&status, 0, sizeof(ADAControllerStatus));
    EventBroker::getInstance().subscribe(this, TOPIC_ADA);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

ADAController::~ADAController()
{
    EventBroker::getInstance().unsubscribe(this);
}

ADAControllerStatus ADAController::getStatus() { return status; }

void ADAController::state_idle(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(IDLE);
        }
        case ADA_CALIBRATE:
        {
            return transition(&ADAController::state_calibrating);
        }
    }
}

void ADAController::state_calibrating(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(CALIBRATING);

            return calibrate();
        }
        case ADA_READY:
        {
            return transition(&ADAController::state_ready);
        }
    }
}

void ADAController::state_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(READY);
        }
        case ADA_CALIBRATE:
        {
            return transition(&ADAController::state_calibrating);
        }
        case FLIGHT_LIFTOFF_DETECTED:
        {
            return transition(&ADAController::state_shadow_mode);
        }
    }
}

void ADAController::state_shadow_mode(const Event& event)
{
    static uint16_t shadowModeTimeoutEventId = -1;

    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SHADOW_MODE);

            shadowModeTimeoutEventId =
                EventBroker::getInstance().postDelayed<SHADOW_MODE_TIMEOUT>(
                    Boardcore::Event{ADA_SHADOW_MODE_TIMEOUT}, TOPIC_ABK);
            break;
        }
        case ADA_SHADOW_MODE_TIMEOUT:
        {
            return transition(&ADAController::state_active);
        }
        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(shadowModeTimeoutEventId);
        }
    }
}

void ADAController::state_active(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(ACTIVE);
        }
        case FLIGHT_APOGEE_DETECTED:
        {
            return transition(&ADAController::state_pressure_stabilization);
        }
    }
}

void ADAController::state_pressure_stabilization(const Event& event)
{
    static uint16_t pressStabTimeoutEventId = -1;

    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(PRESSURE_STABILIZATION);

            pressStabTimeoutEventId =
                EventBroker::getInstance().postDelayed<PRES_STAB_TIMEOUT>(
                    Boardcore::Event{ADA_PRESS_STAB_TIMEOUT}, TOPIC_ADA);
            break;
        }
        case ADA_PRESS_STAB_TIMEOUT:
        {
            return transition(&ADAController::state_drogue_descent);
        }
        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(pressStabTimeoutEventId);
        }
    }
}

void ADAController::state_drogue_descent(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(DROGUE_DESCENT);
        }
        case FLIGHT_DPL_ALT_DETECTED:
        {
            return transition(&ADAController::state_terminal_descent);
        }
    }
}

void ADAController::state_terminal_descent(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(TERMINAL_DESCENT);
        }
        case FLIGHT_LANDING_DETECTED:
        {
            return transition(&ADAController::state_landed);
        }
    }
}

void ADAController::state_landed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(LANDED);
        }
    }
}

void ADAController::logStatus(ADAControllerState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    Logger::getInstance().log(status);
}

void ADAController::calibrate()
{
    // ...
}

}  // namespace Main
