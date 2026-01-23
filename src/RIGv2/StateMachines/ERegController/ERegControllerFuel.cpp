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

#include "ERegControllerFuel.h"

#include <RIGv2/Actuators/ActuatorsData.h>
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

ERegControllerFuel::ERegControllerFuel()
    : FSM{&ERegControllerFuel::state_init, STACK_DEFAULT_FOR_PTHREAD,
          BoardScheduler::eRegControllerPriority()},
      regulator{Config::ERegFuel::STABILIZING_CONFIG,
                Config::ERegFuel::VALVE_INFO, Config::ERegFuel::TARGET_PRESSURE,
                [this]() { return currentSample; },
                [this](float position)
                {
                    getModule<Actuators>()->moveServo(
                        Config::ERegFuel::EREG_SERVO, position);
                    ActuatorsData positionData = {
                        TimestampTimer::getTimestamp(),
                        Config::ERegFuel::EREG_SERVO, position};
                    sdLogger.log(positionData);
                }}
{
    EventBroker::getInstance().subscribe(this, TOPIC_EREG_FUEL);

    changePIDConfig(Config::ERegFuel::STABILIZING_CONFIG,
                    Config::ERegFuel::DISCHARGING_CONFIG);
    changeTargetPressure(Config::ERegFuel::TARGET_PRESSURE);
}

bool ERegControllerFuel::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->eRegFuel();

    uint8_t result = scheduler.addTask([this]() { update(); },
                                       Config::ERegFuel::UPDATE_RATE);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add EReg update task");
        return false;
    }

    if (!FSM::start())
    {
        LOG_ERR(logger, "Failed to start ERegControllerFuel FSM");
        return false;
    }

    return true;
}

void ERegControllerFuel::update()
{
    pressureFilter.add(getModule<Sensors>()->getOxTankPressure().pressure);
    currentSample = pressureFilter.calcMedian();

    if (currentSample > Config::ERegFuel::TARGET_PRESSURE * 1.2)
    {
        EventBroker::getInstance().post(EREG_CLOSE, TOPIC_EREG_OX);

        getModule<Actuators>()->closeServo(Config::ERegFuel::EREG_SERVO);
        getModule<Actuators>()->openServoWithTime(OX_VENTING_VALVE, 5000);
        return;
    }

    if (state == ERegState::PRESSURIZING &&
        abs(currentSample - lastSample) > Config::ERegFuel::PRESSURE_THRESHOLD)
    {
        lastSample = currentSample;
        regulator.update();
        return;
    }

    if (state == ERegState::DISCHARGING)
    {
        lastSample = currentSample;
        regulator.update();
        return;
    }
}

ERegState ERegControllerFuel::getState() { return state; }

void ERegControllerFuel::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ERegState::INIT);

            regulator.init();

            // Immediately transition to ready
            transition(&ERegControllerFuel::state_closed);
            break;
        }
    }
}

void ERegControllerFuel::state_closed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ERegState::CLOSED);

            regulator.end();
            getModule<Actuators>()->closeServo(Config::ERegFuel::EREG_SERVO);
            break;
        }

        case EREG_TOGGLE:
        case EREG_PRESSURIZE:
        {
            transition(&ERegControllerFuel::state_pressurizing);
            break;
        }
    }
}

void ERegControllerFuel::state_pressurizing(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ERegState::PRESSURIZING);

            regulator.changePIDConfig(pressurizationConfig);
            regulator.setReferencePoint(targetPressure);
            regulator.begin();
            lastSample = -1.0f;
            break;
        }

        case EREG_TOGGLE:
        case EREG_CLOSE:
        {
            transition(&ERegControllerFuel::state_closed);
            break;
        }

        case EREG_DISCHARGE:
        {
            transition(&ERegControllerFuel::state_discharging);
            break;
        }
    }
}

void ERegControllerFuel::state_discharging(const Event& event)
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
            transition(&ERegControllerFuel::state_pressurizing);
            break;
        }

        case EREG_TOGGLE:
        case EREG_CLOSE:
        {
            transition(&ERegControllerFuel::state_closed);
            break;
        }
    }
}

void ERegControllerFuel::changePIDConfig(ERegPIDConfig newPressurizationConfig,
                                         ERegPIDConfig newDischargeConfig)
{
    this->pressurizationConfig.KP = newPressurizationConfig.KP;
    this->pressurizationConfig.KI = newPressurizationConfig.KI;
    this->pressurizationConfig.KD = newPressurizationConfig.KD;

    this->dischargeConfig.KP = newDischargeConfig.KP;
    this->dischargeConfig.KI = newDischargeConfig.KI;
    this->dischargeConfig.KD = newDischargeConfig.KD;
}

void ERegControllerFuel::changeTargetPressure(float newTargetPressure)
{
    this->targetPressure = newTargetPressure;
}

void ERegControllerFuel::updateAndLogStatus(ERegState state)
{
    this->state = state;
    // printf("changing to state: %s\n", to_string(state).c_str());
    ERegStateData data = {TimestampTimer::getTimestamp(), state};
    sdLogger.log(data);
}
