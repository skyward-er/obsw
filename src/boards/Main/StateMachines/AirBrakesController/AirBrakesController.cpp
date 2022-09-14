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

#include "AirBrakesController.h"

#include <Main/Actuators/Actuators.h>
#include <Main/BoardScheduler.h>
#include <Main/Configs/ActuatorsConfigs.h>
#include <Main/StateMachines/NASController/NASController.h>
#include <algorithms/AirBrakes/AirBrakesInterp.h>
#include <algorithms/AirBrakes/AirBrakesPI.h>
#include <common/events/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

#ifdef HILMockNAS
#include "Main/Sensors/Sensors.h"
#endif

#ifdef INTERP
#include "Main/Configs/AirBrakesControllerConfigInterp.h"
#include "TrajectorySetInterp.h"
#else
#include "Main/Configs/AirBrakesControllerConfigPI.h"
#include "TrajectorySet.h"
#endif

using namespace std;
using namespace miosix;
using namespace Boardcore;
using namespace Main::AirBrakesControllerConfig;
using namespace Main::ActuatorsConfigs;
using namespace Common;

namespace Main
{

bool AirBrakesController::start()
{
    // Add the update task to the scheduler
    BoardScheduler::getInstance().getScheduler().addTask(
        bind(&AirBrakesController::update, this), UPDATE_PERIOD,
        TaskScheduler::Policy::RECOVER);

    return ActiveObject::start();
}

void AirBrakesController::update()
{
    auto currentPoint =
        TimedTrajectoryPoint{NASController::getInstance().getNasState()};

    if (!abk->isRunning() && status.state == AirBrakesControllerState::ACTIVE &&
        currentPoint.getMac() < MACH_LIMIT)
        abk->begin();

    abk->update();
}

AirBrakesControllerStatus AirBrakesController::getStatus()
{
    PauseKernelLock lock;
    return status;
}

void AirBrakesController::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(AirBrakesControllerState::INIT);

            Actuators::getInstance().setServoAngle(AIR_BRAKES_SERVO, 0);
            Actuators::getInstance().enableServo(AIR_BRAKES_SERVO);

            return transition(&AirBrakesController::state_idle);
        }
    }
}

void AirBrakesController::state_idle(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(AirBrakesControllerState::IDLE);
        }
        case ABK_WIGGLE:
        {
            return wiggleServo();
        }
        case ABK_OPEN:
        {
            Actuators::getInstance().setServoAngle(AIR_BRAKES_SERVO,
                                                   ABK_SERVO_ROTATION);
            break;
        }
        case ABK_RESET:
        {
            Actuators::getInstance().setServoAngle(AIR_BRAKES_SERVO, 0);
            break;
        }
        case FLIGHT_LIFTOFF:
        {
#ifdef INTERP
            abk->setLiftoffTimestamp();
#endif

            return transition(&AirBrakesController::state_shadow_mode);
        }
    }
}

void AirBrakesController::state_shadow_mode(const Event& event)
{
    static uint16_t shadowModeTimeoutEventId = -1;

    switch (event)
    {
        case EV_ENTRY:
        {
            shadowModeTimeoutEventId =
                EventBroker::getInstance().postDelayed<SHADOW_MODE_TIMEOUT>(
                    Boardcore::Event{ABK_SHADOW_MODE_TIMEOUT}, TOPIC_ABK);

            return logStatus(AirBrakesControllerState::SHADOW_MODE);
        }
        case ABK_SHADOW_MODE_TIMEOUT:
        {
            return transition(&AirBrakesController::state_active);
        }
        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(shadowModeTimeoutEventId);
        }
    }
}

void AirBrakesController::state_active(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(AirBrakesControllerState::ACTIVE);
        }
        case FLIGHT_APOGEE_DETECTED:
        case ABK_DISABLE:
        {
            return transition(&AirBrakesController::state_end);
        }
    }
}

void AirBrakesController::state_end(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(AirBrakesControllerState::END);

            abk->end();
            Actuators::getInstance().setServoAngle(AIR_BRAKES_SERVO, 0);

            return;
        }
    }
}

AirBrakesController::AirBrakesController()
    : FSM(&AirBrakesController::state_init)
{
#ifndef INTERP
    this->abk = new AirBrakesPI(
#ifndef HILMockNAS
        []() {
            return TimedTrajectoryPoint{
                NASController::getInstance().getNasState()};
        },
#else   // HILMockNAS
        []() { return Sensors::getInstance().state.kalman->getLastSample(); },
#endif  // HILMockNAS
        TRAJECTORY_SET, AirBrakesControllerConfig::ABK_CONFIG,
        AirBrakesControllerConfig::ABK_CONFIG_PI,
        [](float position) {
            Actuators::getInstance().setServo(ServosList::AIR_BRAKES_SERVO,
                                              position);
        });
#else  // INTERP
    this->abk = new AirBrakesInterp(
#ifndef HILMockNAS
        []() {
            return TimedTrajectoryPoint{
                NASController::getInstance().getNasState()};
        },
#else   // HILMockNAS
        []() { return Sensors::getInstance().state.kalman->getLastSample(); },
#endif  // HILMockNAS
        TRAJECTORY_SET, AirBrakesControllerConfig::ABK_CONFIG,
        AirBrakesControllerConfig::ABK_CONFIG_INTERP,
        [](float position) {
            Actuators::getInstance().setServo(ServosList::AIR_BRAKES_SERVO,
                                              position);
        },
        Main::dz);
#endif  // INTERP

    EventBroker::getInstance().subscribe(this, TOPIC_ABK);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

AirBrakesController::~AirBrakesController()
{
    EventBroker::getInstance().unsubscribe(this);
}

void AirBrakesController::logStatus(AirBrakesControllerState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    Logger::getInstance().log(status);
}

void AirBrakesController::wiggleServo()
{
    for (int i = 0; i < 2; i++)
    {
        Actuators::getInstance().setServoAngle(AIR_BRAKES_SERVO,
                                               ABK_SERVO_ROTATION);
        miosix::Thread::sleep(500);
        Actuators::getInstance().setServoAngle(AIR_BRAKES_SERVO, 0);
        miosix::Thread::sleep(500);
    }
}

}  // namespace Main
