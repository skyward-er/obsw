/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Pietro Bortolus
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

#include "ERegControllerFUEL.h"

#include <RIGv2/BoardScheduler.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

#include <chrono>

using namespace std::chrono;
using namespace Boardcore;
using namespace RIGv2;
using namespace miosix;
using namespace Common;

ERegControllerFUEL::ERegControllerFUEL()
    : FSM{&ERegControllerFUEL::state_init, STACK_DEFAULT_FOR_PTHREAD,
          BoardScheduler::eRegControllerPriority()},
      regulator{Config::ERegFUEL::STABILIZING_CONFIG,
                Config::ERegFUEL::TARGET_PRESSURE,
                [this]() { return pressureFilter.calcMedian(); },
                [this](float position) {
                    getModule<Actuators>()->moveServo(
                        Config::ERegFUEL::EREG_SERVO, position);
                }}
{
    EventBroker::getInstance().subscribe(this, TOPIC_EREG_FUEL);

    changePIDConfig(Config::ERegFUEL::STABILIZING_CONFIG,
                    Config::ERegFUEL::DISCHARGING_CONFIG);
    changeTargetPressure(Config::ERegFUEL::TARGET_PRESSURE);
}

bool ERegControllerFUEL::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->eRegFuel();

    uint8_t result = scheduler.addTask([this]() { update(); },
                                       Config::ERegFUEL::UPDATE_RATE);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add EReg update task");
        return false;
    }

    if (!FSM::start())
    {
        LOG_ERR(logger, "Failed to start ERegControllerFUEL FSM");
        return false;
    }

    return true;
}

void ERegControllerFUEL::update()
{
    pressureFilter.add(getModule<Sensors>()->getFuelTankPressure().pressure);

    if (pressureFilter.calcMedian() > Config::ERegFUEL::TARGET_PRESSURE * 1.5)
    {
        EventBroker::getInstance().post(EREG_CLOSE, TOPIC_EREG_FUEL);

        getModule<Actuators>()->closeServo(Config::ERegFUEL::EREG_SERVO);
        getModule<Actuators>()->openServoWithTime(FUEL_VENTING_VALVE, 5000);
    }

    if (state == ERegState::PRESSURIZING || state == ERegState::DISCHARGING)
        regulator.update();
}

ERegState ERegControllerFUEL::getState() { return state; }

void ERegControllerFUEL::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ERegState::INIT);

            regulator.init();

            // Immediately transition to ready
            transition(&ERegControllerFUEL::state_closed);
            break;
        }
    }
}

void ERegControllerFUEL::state_closed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ERegState::CLOSED);

            regulator.end();
            getModule<Actuators>()->closeServo(Config::ERegFUEL::EREG_SERVO);
            break;
        }

        case EREG_TOGGLE:
        case EREG_PRESSURIZE:
        {
            transition(&ERegControllerFUEL::state_pressurizing);
            break;
        }
    }
}

void ERegControllerFUEL::state_pressurizing(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ERegState::PRESSURIZING);

            regulator.changePIDConfig(pressurizationConfig);
            regulator.setReferencePoint(targetPressure);
            regulator.begin();
            break;
        }

        case EREG_TOGGLE:
        case EREG_CLOSE:
        {
            transition(&ERegControllerFUEL::state_closed);
            break;
        }

        case EREG_DISCHARGE:
        {
            transition(&ERegControllerFUEL::state_discharging);
            break;
        }
    }
}

void ERegControllerFUEL::state_discharging(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ERegState::DISCHARGING);

            regulator.changePIDConfig(dischargeConfig);
            break;
        }

        case EREG_PRESSURIZE:
        {
            transition(&ERegControllerFUEL::state_pressurizing);
            break;
        }

        case EREG_TOGGLE:
        case EREG_CLOSE:
        {
            transition(&ERegControllerFUEL::state_closed);
            break;
        }
    }
}

void ERegControllerFUEL::changePIDConfig(ERegPIDConfig newPressurizationConfig,
                                         ERegPIDConfig newDischargeConfig)
{
    this->pressurizationConfig = newPressurizationConfig;
    this->dischargeConfig      = newDischargeConfig;
}

void ERegControllerFUEL::changeTargetPressure(float newTargetPressure)
{
    this->targetPressure = newTargetPressure;
}

void ERegControllerFUEL::updateAndLogStatus(ERegState state)
{
    this->state = state;
    // printf("changing to state: %s\n", to_string(state).c_str());
    ERegStateData data = {TimestampTimer::getTimestamp(), state};
    sdLogger.log(data);
}
