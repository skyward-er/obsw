/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include "ABKController.h"

#include <Main/Configs/ABKConfig.h>
#include <Main/Configs/SchedulerConfig.h>
#include <algorithms/ABK/ABKData.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <events/EventBroker.h>

#include <chrono>

using namespace std::chrono;
using namespace Boardcore;
using namespace Main;
using namespace miosix;
using namespace Common;

ABKController::ABKController()
    : FSM{&ABKController::state_init, STACK_DEFAULT_FOR_PTHREAD,
          Config::Scheduler::ABK_PRIORITY}

{
    EventBroker::getInstance().subscribe(this, TOPIC_ABK);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

bool ABKController::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->getAbkScheduler();

    abkTaskId =
        scheduler.addTask([this]() { update(); }, Config::ABK::UPDATE_RATE);

    if (abkTaskId == 0)
    {
        LOG_ERR(logger, "Failed to add ABK update task");
        return false;
    }

    if (!FSM::start())
    {
        LOG_ERR(logger, "Failed to start ABK FSM");
        return false;
    }

    // Disable the task for now
    scheduler.disableTask(abkTaskId);
    return true;
}

ABKControllerState ABKController::getState() { return state; }

void ABKController::update()
{
    if (state == ABKControllerState::ACTIVE)
    {
        auto anasState = getModule<NASController>()->getANASState();
        ABKIn input{.MEAMass      = getModule<MotorStatus>()->getMeaMass(),
                    .ANASPosition = {anasState.n, anasState.e, anasState.d},
                    .ANASVelocity = {anasState.vn, anasState.ve, anasState.vd}};

        abk.setABK_In(input);

        abk.step();

        // move the aerobrakes
        getModule<Actuators>()->setAbkPosition(abk.getABK_Control());

        // log the data
        ABKLogsData logs{TimestampTimer::getTimestamp(),
                         abk.getABK_Logs_OBSW()};
        sdLogger.log(logs);
    }
}

void ABKController::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ABKControllerState::INIT);
            abk.initialize();

            // Immediately transition to ready
            transition(&ABKController::state_ready);
            break;
        }
    }
}

void ABKController::state_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ABKControllerState::READY);
            break;
        }

        case FLIGHT_ARMED:
        {
            transition(&ABKController::state_armed);
            break;
        }
    }
}

void ABKController::state_armed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ABKControllerState::ARMED);
            break;
        }

        case FLIGHT_DISARMED:
        {
            transition(&ABKController::state_ready);
            break;
        }

        case FLIGHT_MOTOR_SHUTDOWN:
        {
            // Enable only after motor shutdown
            transition(&ABKController::state_shadow_mode);
            break;
        }
    }
}

void ABKController::state_shadow_mode(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ABKControllerState::SHADOW_MODE);

            shadowModeTimeoutEvent = EventBroker::getInstance().postDelayed(
                ABK_SHADOW_MODE_TIMEOUT, TOPIC_ABK,
                milliseconds{Config::ABK::SHADOW_MODE_TIMEOUT}.count());
            break;
        }

        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(shadowModeTimeoutEvent);
            break;
        }

        case ABK_SHADOW_MODE_TIMEOUT:
        {
            transition(&ABKController::state_waiting_mach);
            break;
        }

        case FLIGHT_APOGEE_DETECTED:
        case FLIGHT_LANDING_DETECTED:
        {
            transition(&ABKController::state_end);
            break;
        }
    }
}

void ABKController::state_active(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ABKControllerState::ACTIVE);
            getModule<BoardScheduler>()->getAbkScheduler().enableTask(
                abkTaskId);

            break;
        }

        case FLIGHT_APOGEE_DETECTED:
        case FLIGHT_LANDING_DETECTED:
        {
            transition(&ABKController::state_end);
            break;
        }
    }
}

void ABKController::state_end(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ABKControllerState::END);

            // Stop the update task
            getModule<BoardScheduler>()->getAbkScheduler().disableTask(
                abkTaskId);

            // Close the airbrakes
            getModule<Actuators>()->setAbkPosition(0.0f);
            break;
        }
    }
}

void ABKController::updateAndLogStatus(ABKControllerState state)
{
    this->state              = state;
    ABKControllerStatus data = {TimestampTimer::getTimestamp(), state};
    sdLogger.log(data);
}
