/* Copyright (c) 2026 Skyward Experimental Rocketry
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

#include "SDAController.h"

#include <Main/Configs/SDAConfig.h>
#include <Main/Configs/SchedulerConfig.h>
#include <algorithms/SDA/SDAData.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <events/EventBroker.h>

#include <chrono>

using namespace std::chrono;
using namespace Main;
using namespace Boardcore;
using namespace Common;

SDAController::SDAController()
    : FSM{&SDAController::state_init, miosix::STACK_DEFAULT_FOR_PTHREAD,
          Config::Scheduler::SDA_PRIORITY},
      initialMass{Config::SDA::DEFAULT_INITIAL_ROCKET_MASS},
      minBurnTime{Config::SDA::SHADOW_MODE_TIMEOUT},
      apogeeTarget{Config::SDA::SHUTDOWN_APOGEE_TARGET}, sda{}
{
    EventBroker::getInstance().subscribe(this, TOPIC_SDA);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

bool SDAController::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->getSdaScheduler();

    uint8_t result =
        scheduler.addTask([this]() { update(); }, Config::SDA::UPDATE_RATE);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add SDA update task");
        return false;
    }

    if (!FSM::start())
    {
        LOG_ERR(logger, "Failed to start SDA FSM");
        return false;
    }

    return true;
}

bool SDAController::getSDAOutput()
{
    Lock<FastMutex> lock{sdaMutex};
    return sda.getSDA_Shutdown();
}

SDAControllerState SDAController::getState() { return state; }

float SDAController::getInitialMass() { return initialMass.load(); }

milliseconds SDAController::getMinBurnTime() { return minBurnTime.load(); }

void SDAController::setMinBurnTime(milliseconds time) { minBurnTime = time; }

float SDAController::getApogeeTarget() { return apogeeTarget.load(); }

void SDAController::setApogeeTarget(float apogee) { apogeeTarget = apogee; }

void SDAController::update()
{
    SDAControllerState curState = state;

    // Lock SDA for the whole duration of the update
    Lock<FastMutex> lock{sdaMutex};

    if (curState == SDAControllerState::ARMED ||
        curState == SDAControllerState::SHADOW_MODE ||
        curState == SDAControllerState::ACTIVE ||
        curState == SDAControllerState::ACTIVE_UNPOWERED)
    {
        auto anasState = getModule<NASController>()->getANASState();

        SDAIn input = {
            .MEAMass      = getModule<MotorStatus>()->getMeaMass(),
            .ANASPosition = {anasState.n, anasState.e, anasState.d},
            .ANASVelocity = {anasState.vn, anasState.ve, anasState.vd},

        };

        sda.setSDA_IN(input);
        sda.step();

        bool shutdown = sda.getSDA_Shutdown();

        if (curState == SDAControllerState::ACTIVE && shutdown)
        {
            // Throw events only in ACTIVE
            EventBroker::getInstance().post(SDA_SHUTDOWN_DETECTED, TOPIC_SDA);
            getModule<StatsRecorder>()->
        }

        SDALogsWrapper logData{sda.getSDA_Logs_OBSW()};
        logData.Timestamp = TimestampTimer::getTimestamp();

        sdLogger.log(logData);
    }
}

void SDAController::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(SDAControllerState::INIT);

            // Immediately transition to ready
            transition(&SDAController::state_ready);
            break;
        }
    }
}

void SDAController::state_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(SDAControllerState::READY);
            break;
        }

        case SDA_FORCE_START:
        case FLIGHT_ARMED:
        {
            transition(&SDAController::state_armed);
            break;
        }
    }
}

void SDAController::state_armed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(SDAControllerState::ARMED);
            break;
        }

        case SDA_FORCE_STOP:
        case FLIGHT_DISARMED:
        {
            transition(&SDAController::state_ready);
            break;
        }

        case FLIGHT_LIFTOFF:
        {
            transition(&SDAController::state_shadow_mode);
            break;
        }
    }
}

void SDAController::state_shadow_mode(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(SDAControllerState::SHADOW_MODE);

            auto shadowModeDelay =
                getModule<AlgoReference>()->computeTimeSinceLiftoff(
                    minBurnTime);

            shadowModeTimeoutEvent = EventBroker::getInstance().postDelayed(
                SDA_SHADOW_MODE_TIMEOUT, TOPIC_SDA,
                milliseconds{shadowModeDelay}.count());
            break;
        }

        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(shadowModeTimeoutEvent);
            break;
        }

        case SDA_FORCE_STOP:
        {
            transition(&SDAController::state_ready);
            break;
        }

        case SDA_SHADOW_MODE_TIMEOUT:
        {
            transition(&SDAController::state_active);
            break;
        }

        case FLIGHT_MOTOR_SHUTDOWN:
        {
            transition(&SDAController::state_active_unpowered);
            break;
        }

        case FLIGHT_LANDING_DETECTED:
        {
            transition(&SDAController::state_end);
            break;
        }
    }
}

void SDAController::state_active(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(SDAControllerState::ACTIVE);
            break;
        }

        case SDA_FORCE_STOP:
        {
            transition(&SDAController::state_ready);
            break;
        }

        case FLIGHT_MOTOR_SHUTDOWN:
        {
            transition(&SDAController::state_active_unpowered);
            break;
        }

        case FLIGHT_LANDING_DETECTED:
        {
            transition(&SDAController::state_end);
            break;
        }
    }
}

void SDAController::state_active_unpowered(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(SDAControllerState::ACTIVE_UNPOWERED);
            break;
        }

        case SDA_FORCE_STOP:
        {
            transition(&SDAController::state_ready);
            break;
        }

        case FLIGHT_APOGEE_DETECTED:
        case FLIGHT_LANDING_DETECTED:
        {
            transition(&SDAController::state_end);
            break;
        }
    }
}

void SDAController::state_end(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(SDAControllerState::END);
            break;
        }
    }
}

void SDAController::updateAndLogStatus(SDAControllerState state)
{
    this->state              = state;
    SDAControllerStatus data = {TimestampTimer::getTimestamp(), state};
    sdLogger.log(data);
}
