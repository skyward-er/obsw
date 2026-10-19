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

#include "CATSEController.h"

#include <Main/Configs/CATSEConfig.h>
#include <Main/Configs/SchedulerConfig.h>
#include <algorithms/AlgorithmsData.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <events/EventBroker.h>

#include <chrono>

using namespace std::chrono;
using namespace Main;
using namespace Boardcore;
using namespace Common;

CATSEController::CATSEController()
    : FSM{&CATSEController::state_init, miosix::STACK_DEFAULT_FOR_PTHREAD,
          Config::Scheduler::CATSE_PRIORITY},
      cats{}
{
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    EventBroker::getInstance().subscribe(this, TOPIC_CATSE);
}

bool CATSEController::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->getCatseScheduler();

    uint8_t result =
        scheduler.addTask([this]() { update(); }, Config::CATSE::UPDATE_RATE);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add CATSE update task");
        return false;
    }

    if (!FSM::start())
    {
        LOG_ERR(logger, "Failed to start CATSE FSM");
        return false;
    }

    return true;
}

CATSEControllerState CATSEController::getState() { return state; }

CATSEState CATSEController::getCATSEState()
{
    Lock<FastMutex> lock{catseMutex};
    return catseState;
}

void CATSEController::update()
{
    CATSEControllerState curState = state;

    if (curState == CATSEControllerState::RUNNING)
    {
        auto baro = getModule<Sensors>()->getAtmosPressureLastSample();
        auto imu  = getModule<Sensors>()->getIMULastSample();

        CATSSensors input = {
            .Accelerometer = {imu.accelerationX, imu.accelerationY,
                              imu.accelerationZ},
            .Gyroscope     = {imu.angularSpeedX, imu.angularSpeedY,
                              imu.angularSpeedZ},
            .Barometer     = baro.pressure,
        };

        cats.setCATSSensors(input);
        cats.step();

        Lock<FastMutex> lock{catseMutex};
        catseState =
            CATSEState{TimestampTimer::getTimestamp(), cats.getKalman_Out()};
        sdLogger.log(catseState);
    }
}

void CATSEController::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(CATSEControllerState::INIT);

            cats.initialize();

            // Immediately transition to ready
            transition(&CATSEController::state_running);
            break;
        }
    }
}

void CATSEController::state_running(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(CATSEControllerState::RUNNING);
            break;
        }

        case CATSE_RESET:
        {
            transition(&CATSEController::state_init);
            break;
        }

        case FLIGHT_LANDING_DETECTED:
        {
            transition(&CATSEController::state_end);
            break;
        }
    }
}
void CATSEController::state_end(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(CATSEControllerState::END);
            break;
        }
    }
}

void CATSEController::updateAndLogStatus(CATSEControllerState state)
{
    this->state                = state;
    CATSEControllerStatus data = {TimestampTimer::getTimestamp(), state};
    sdLogger.log(data);
}
