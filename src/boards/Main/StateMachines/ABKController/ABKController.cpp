/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <Main/Actuators/Actuators.h>
#include <Main/Configs/ABKConfig.h>
#include <Main/StateMachines/ABKController/ABKController.h>
#include <Main/StateMachines/ABKController/TrajectorySet.h>
#include <Main/StateMachines/MEAController/MEAController.h>
#include <Main/StateMachines/NASController/NASController.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

using namespace Boardcore;
using namespace Main::ABKTrajectories;
using namespace Common;

namespace Main
{

ABKController::ABKController(TaskScheduler* sched)
    : FSM(&ABKController::state_init),
      abk(
          []()
          {
              return TimedTrajectoryPoint(ModuleManager::getInstance()
                                              .get<NASController>()
                                              ->getNasState());
          },
          OPEN_TRAJECTORY_SET, CLOSED_TRAJECTORY_SET, ABKConfig::ABK_CONFIG,
          getConfig(),
          [](float position)
          {
              ModuleManager::getInstance().get<Actuators>()->setServoPosition(
                  ServosList::AIR_BRAKES_SERVO, position);
          }),
      scheduler(sched)
{
    EventBroker::getInstance().subscribe(this, TOPIC_ABK);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

bool ABKController::start()
{
    // Insert the update task into the task scheduler
    size_t result =
        scheduler->addTask([&]() { update(); }, ABKConfig::UPDATE_PERIOD,
                           TaskScheduler::Policy::RECOVER);

    return result != 0 && ActiveObject::start();
}

void ABKController::update()
{
    // No need for pause kernel due to its presence inside the getter
    ABKControllerStatus status = getStatus();

    if (!abk.isRunning() && status.state == ABKControllerState::ACTIVE)
    {
        // Begin the algorithm with the last estimated mass
        abk.begin(ModuleManager::getInstance()
                      .get<MEAController>()
                      ->getStatus()
                      .estimatedMass);
    }

    // Update the algorithm if in Active mode
    if (status.state == ABKControllerState::ACTIVE)
    {
        abk.update();
    }
}

ABKControllerStatus ABKController::getStatus()
{
    miosix::PauseKernelLock lock;
    return status;
}

void ABKController::state_init(const Event& event)
{
    ModuleManager& modules = ModuleManager::getInstance();
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(ABKControllerState::INIT);

            // Reset the servo position
            modules.get<Actuators>()->setServoPosition(
                ServosList::AIR_BRAKES_SERVO, 0);

            return transition(&ABKController::state_idle);
        }
    }
}

void ABKController::state_idle(const Event& event)
{
    static uint16_t delayTimeoutEventId = 0;

    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(ABKControllerState::IDLE);
        }
        case FLIGHT_MOTOR_SHUTDOWN:
        {
            // Wait a fixed time to start the ABK algorithm, in order to let the
            // MEA algorithm estimate correctly the mass
            delayTimeoutEventId = EventBroker::getInstance().postDelayed(
                ABK_SHADOW_MODE_TIMEOUT, TOPIC_ABK, ABKConfig::DELAY_TIMEOUT);
            break;
        }
        case ABK_SHADOW_MODE_TIMEOUT:
        {
            return transition(&ABKController::state_active);
        }
        case FLIGHT_LANDING_DETECTED:
        {
            return transition(&ABKController::state_end);
        }
        case EV_EXIT:
        {
            // Remove the shadow mode event. This works even though the event is
            // expired (aka after shadow_mode_timeout) because the event broker
            // assigns a progressive number for every delayed event. If and only
            // if the number of registered delayed event is less than 2^16, then
            // this technique is valid.
            return EventBroker::getInstance().removeDelayed(
                delayTimeoutEventId);
        }
    }
}

void ABKController::state_active(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(ABKControllerState::ACTIVE);
        }
        case FLIGHT_LANDING_DETECTED:
        case FLIGHT_APOGEE_DETECTED:
        {
            return transition(&ABKController::state_end);
        }
    }
}

void ABKController::state_end(const Event& event)
{
    ModuleManager& modules = ModuleManager::getInstance();
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(ABKControllerState::END);

            // End the ABK (TODO Maybe a mutex?)
            abk.end();

            // Close the servo
            modules.get<Actuators>()->setServoPosition(
                ServosList::AIR_BRAKES_SERVO, 0);

            return;
        }
    }
}

void ABKController::logStatus(ABKControllerState state)
{
    {
        miosix::PauseKernelLock lock;
        status.timestamp = TimestampTimer::getTimestamp();
        status.state     = state;
    }

    Logger::getInstance().log(status);
}

AirBrakesInterpConfig ABKController::getConfig()
{
    AirBrakesInterpConfig config;

    config.ABK_CRITICAL_ALTITUDE   = ABKConfig::ABK_CRITICAL_ALTITUDE;
    config.STARTING_FILTER_VALUE   = ABKConfig::STARTING_FILTER_VALUE;
    config.FILTER_MAXIMUM_ALTITUDE = ABKConfig::MAXIMUM_ALTITUDE;
    config.FILTER_MINIMUM_ALTITUDE = ABKConfig::MINIMUM_ALTITUDE;
    config.INITIAL_MASS            = ABKConfig::INITIAL_MASS;
    config.N_FORWARD               = ABKConfig::N_FORWARD;
    config.DM                      = ABKConfig::DM;
    config.DZ                      = ABKConfig::DZ;

    return config;
}

}  // namespace Main