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

#include "EregControllerOx.h"

#include <RIGv3/Actuators/ActuatorsData.h>
#include <RIGv3/BoardScheduler.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

#include <chrono>

using namespace std::chrono;
using namespace Boardcore;
using namespace RIGv3;
using namespace miosix;
using namespace Common;

EregControllerOx::EregControllerOx()
    : FSM{&EregControllerOx::state_init, STACK_DEFAULT_FOR_PTHREAD,
          BoardScheduler::eregControllerPriority()},
      regulator{
          Config::EregOx::STABILIZING_CONFIG,
          Config::EregOx::VALVE_INFO,
          Config::EregOx::TARGET_PRESSURE,
      }
{
    EventBroker::getInstance().subscribe(this, TOPIC_FIRING_SEQUENCE);
    EventBroker::getInstance().subscribe(this, TOPIC_EREG_OX);
}

bool EregControllerOx::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->eregOx();

    uint8_t result =
        scheduler.addTask([this]() { update(); }, Config::EregOx::UPDATE_RATE);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add EReg update task");
        return false;
    }

    if (!FSM::start())
    {
        LOG_ERR(logger, "Failed to start EregControllerOx FSM");
        return false;
    }

    return true;
}

void EregControllerOx::update()
{
    EregOxData logData;

    logData.downstreamPressure =
        getModule<Sensors>()->getOxTankPressure().pressure;
    logData.upstreamPressure =
        getModule<Sensors>()->getPrzTankPressure().pressure;

    downstreamPressureFilter.add(logData.downstreamPressure);
    upstreamPressureFilter.add(logData.upstreamPressure);

    logData.filteredDownstreamPressure = downstreamPressureFilter.calcMean();
    logData.filteredUpstreamPressure   = upstreamPressureFilter.calcMean();
    logData.timestamp                  = TimestampTimer::getTimestamp();

    if (logData.filteredDownstreamPressure > targetPressure * 1.2)
    {
        EventBroker::getInstance().post(EREG_CLOSE, TOPIC_EREG_OX);

        // TODO: put venting valve in the config
        getModule<Actuators>()->closeValve(Config::EregOx::EREG_SERVO);
        getModule<Actuators>()->openValveWithTime(ServosList::OX_VENTING_VALVE,
                                                  5000);
        return;
    }

    if (state == EregState::PRESSURIZING &&
        (abs(logData.filteredDownstreamPressure - lastDownstreamInput) >
             Config::EregOx::PRESSURE_THRESHOLD ||
         abs(logData.filteredUpstreamPressure - lastUpstreamInput) >
             Config::EregOx::PRESSURE_THRESHOLD))
    {
        lastDownstreamInput = logData.filteredDownstreamPressure;
        lastUpstreamInput   = logData.filteredUpstreamPressure;
        regulator.setInput(logData.filteredDownstreamPressure,
                           logData.filteredUpstreamPressure);

        regulator.update();

        logData.servoPosition = regulator.getOutput();
        getModule<Actuators>()->moveValve(Config::EregOx::EREG_SERVO,
                                          logData.servoPosition);
        sdLogger.log(logData);
        return;
    }

    if (state == EregState::PILOTFLAME || state == EregState::FIRING)
    {
        regulator.setInput(logData.filteredDownstreamPressure,
                           logData.filteredUpstreamPressure);

        regulator.update();

        logData.servoPosition = regulator.getOutput();
        getModule<Actuators>()->moveValve(Config::EregOx::EREG_SERVO,
                                          logData.servoPosition);
        sdLogger.log(logData);
        return;
    }
}

EregState EregControllerOx::getState() { return state; }

void EregControllerOx::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(EregState::INIT);

            regulator.init();

            // Immediately transition to closed state
            transition(&EregControllerOx::state_closed);
            break;
        }
    }
}

void EregControllerOx::state_closed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(EregState::CLOSED);

            regulator.end();
            getModule<Actuators>()->closeValve(Config::EregOx::EREG_SERVO);
            break;
        }

        case EREG_TOGGLE:
        case EREG_PRESSURIZE:
        {
            transition(&EregControllerOx::state_pressurizing);
            break;
        }
    }
}

void EregControllerOx::state_pressurizing(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(EregState::PRESSURIZING);

            regulator.setReferencePoint(targetPressure);
            regulator.changePIDConfig(pressurizationConfig);
            lastDownstreamInput = -1.0f;
            lastUpstreamInput   = -1.0f;
            regulator.begin();
            break;
        }

        case EREG_TOGGLE:
        case EREG_CLOSE:
        case FIRING_SEQUENCE_ABORT:
        {
            transition(&EregControllerOx::state_closed);
            break;
        }

        case FIRING_SEQUENCE_IGNITER_OK:
        {
            transition(&EregControllerOx::state_pilot_flame);
            break;
        }
    }
}

void EregControllerOx::state_pilot_flame(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(EregState::PILOTFLAME);

            regulator.setReferencePoint(targetPressure);
            regulator.changePIDConfig(dischargeConfig);
            regulator.setIntegralContribution(pilotFlameIntegral);
            break;
        }

        case FIRING_SEQUENCE_PILOT_FLAME_OK:
        {
            transition(&EregControllerOx::state_firing);
            break;
        }

        case EREG_TOGGLE:
        case EREG_CLOSE:
        case FIRING_SEQUENCE_ABORT:
        {
            transition(&EregControllerOx::state_closed);
            break;
        }
    }
}

void EregControllerOx::state_firing(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(EregState::FIRING);

            regulator.changePIDConfig(dischargeConfig);
            regulator.setIntegralContribution(rampupIntegral);
            break;
        }

        case EREG_TOGGLE:
        case EREG_CLOSE:
        case FIRING_SEQUENCE_ABORT:
        {
            transition(&EregControllerOx::state_closed);
            break;
        }
    }
}

void EregControllerOx::changePIDConfig(EregPIDConfig newPressurizationConfig,
                                       EregPIDConfig newDischargeConfig)
{
    this->pressurizationConfig.KP = newPressurizationConfig.KP;
    this->pressurizationConfig.KI = newPressurizationConfig.KI;
    this->pressurizationConfig.KD = newPressurizationConfig.KD;

    this->dischargeConfig.KP = newDischargeConfig.KP;
    this->dischargeConfig.KI = newDischargeConfig.KI;
    this->dischargeConfig.KD = newDischargeConfig.KD;
}

void EregControllerOx::changeTargetPressure(float newTargetPressure)
{
    this->targetPressure = newTargetPressure;
}

void EregControllerOx::setIntegralContribution(float newPilotFlameIntegral,
                                               float newRampupIntegral)
{
    this->pilotFlameIntegral = newPilotFlameIntegral;
    this->rampupIntegral     = newRampupIntegral;
}

void EregControllerOx::updateAndLogStatus(EregState state)
{
    this->state = state;
    // printf("changing to state: %s\n", to_string(state).c_str());
    EregOxStateData data = {TimestampTimer::getTimestamp(), state};
    sdLogger.log(data);
}
