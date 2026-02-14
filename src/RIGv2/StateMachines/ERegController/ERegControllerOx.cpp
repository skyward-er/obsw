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

#include "ERegControllerOx.h"

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

ERegControllerOx::ERegControllerOx()
    : FSM{&ERegControllerOx::state_init, STACK_DEFAULT_FOR_PTHREAD,
          BoardScheduler::eRegControllerPriority()},
      regulator{
          Config::ERegOx::STABILIZING_CONFIG,
          Config::ERegOx::VALVE_INFO,
          Config::ERegOx::TARGET_PRESSURE,
      }
{
    EventBroker::getInstance().subscribe(this, TOPIC_EREG_OX);

    changePIDConfig(Config::ERegOx::STABILIZING_CONFIG,
                    Config::ERegOx::DISCHARGING_CONFIG);
    changeTargetPressure(Config::ERegOx::TARGET_PRESSURE);
}

bool ERegControllerOx::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->eRegOx();

    uint8_t result =
        scheduler.addTask([this]() { update(); }, Config::ERegOx::UPDATE_RATE);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add EReg update task");
        return false;
    }

    if (!FSM::start())
    {
        LOG_ERR(logger, "Failed to start ERegControllerOx FSM");
        return false;
    }

    return true;
}

void ERegControllerOx::update()
{
    downstreamPressureFilter.add(
        getModule<Sensors>()->getOxTankPressure().pressure);

    upstreamPressureFilter.add(
        getModule<Sensors>()->getPrzTankPressure().pressure);

    downstreamSample = downstreamPressureFilter.calcMedian();
    upstreamSample   = upstreamPressureFilter.calcMedian();

    pidData.timestamp          = TimestampTimer::getTimestamp();
    pidData.downstreamPressure = downstreamSample;
    pidData.upstreamPressure   = upstreamSample;

    if (downstreamSample > Config::ERegOx::TARGET_PRESSURE * 1.2)
    {
        EventBroker::getInstance().post(EREG_CLOSE, TOPIC_EREG_OX);

        getModule<Actuators>()->closeServo(Config::ERegOx::EREG_SERVO);
        getModule<Actuators>()->openServoWithTime(OX_VENTING_VALVE, 5000);
        return;
    }

    if (state == ERegState::PRESSURIZING &&
        (abs(downstreamSample - lastDownstreamSample) >
             Config::ERegOx::PRESSURE_THRESHOLD ||
         abs(upstreamSample - lastUpstreamSample) >
             Config::ERegOx::PRESSURE_THRESHOLD))
    {
        lastDownstreamSample = downstreamSample;
        lastUpstreamSample   = upstreamSample;
        regulator.setInput(downstreamSample, upstreamSample);

        regulator.update();

        pidData.servoPosition = regulator.getOutput();
        getModule<Actuators>()->moveServo(Config::ERegOx::EREG_SERVO,
                                          pidData.servoPosition);
        sdLogger.log(pidData);
        return;
    }

    if (state == ERegState::DISCHARGING)
    {
        lastDownstreamSample = downstreamSample;
        lastUpstreamSample   = upstreamSample;
        regulator.setInput(downstreamSample, upstreamSample);

        regulator.update();

        pidData.servoPosition = regulator.getOutput();
        getModule<Actuators>()->moveServo(Config::ERegOx::EREG_SERVO,
                                          pidData.servoPosition);
        sdLogger.log(pidData);
        return;
    }
}

ERegState ERegControllerOx::getState() { return state; }

void ERegControllerOx::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ERegState::INIT);

            regulator.init();

            // Immediately transition to ready
            transition(&ERegControllerOx::state_closed);
            break;
        }
    }
}

void ERegControllerOx::state_closed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ERegState::CLOSED);

            regulator.end();
            getModule<Actuators>()->closeServo(Config::ERegOx::EREG_SERVO);
            break;
        }

        case EREG_TOGGLE:
        case EREG_PRESSURIZE:
        {
            transition(&ERegControllerOx::state_pressurizing);
            break;
        }
    }
}

void ERegControllerOx::state_pressurizing(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ERegState::PRESSURIZING);

            regulator.changePIDConfig(pressurizationConfig);
            regulator.setReferencePoint(targetPressure);
            regulator.begin();
            lastDownstreamSample = -1.0f;
            lastUpstreamSample   = -1.0f;
            break;
        }

        case EREG_TOGGLE:
        case EREG_CLOSE:
        {
            transition(&ERegControllerOx::state_closed);
            break;
        }

        case EREG_DISCHARGE:
        {
            transition(&ERegControllerOx::state_discharging);
            break;
        }
    }
}

void ERegControllerOx::state_discharging(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ERegState::DISCHARGING);

            regulator.changePIDConfig(dischargeConfig);
            break;
        }

        case EREG_TOGGLE:
        case EREG_CLOSE:
        {
            transition(&ERegControllerOx::state_closed);
            break;
        }
    }
}

void ERegControllerOx::changePIDConfig(ERegPIDConfig newPressurizationConfig,
                                       ERegPIDConfig newDischargeConfig)
{
    this->pressurizationConfig.KP = newPressurizationConfig.KP;
    this->pressurizationConfig.KI = newPressurizationConfig.KI;
    this->pressurizationConfig.KD = newPressurizationConfig.KD;

    this->dischargeConfig.KP = newDischargeConfig.KP;
    this->dischargeConfig.KI = newDischargeConfig.KI;
    this->dischargeConfig.KD = newDischargeConfig.KD;
}

void ERegControllerOx::changeTargetPressure(float newTargetPressure)
{
    this->targetPressure = newTargetPressure;
}

void ERegControllerOx::updateAndLogStatus(ERegState state)
{
    this->state = state;
    // printf("changing to state: %s\n", to_string(state).c_str());
    ERegStateData data = {TimestampTimer::getTimestamp(), state};
    sdLogger.log(data);
}
