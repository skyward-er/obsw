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
#include <Main/Configs/ActuatorsConfigs.h>
#include <Main/Configs/AirBrakesControllerConfig.h>
#include <Main/StateMachines/NASController/NASController.h>
#include <Main/events/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

#include "RoccarasoTrajectorySet.h"

using namespace miosix;
using namespace Boardcore;
using namespace Main::AirBrakesControllerConfigs;
using namespace Main::ActuatorsConfigs;

namespace Main
{

void AirBrakesController::update()
{
    abk.update();

#ifdef HILSimulation
    // in order to respond always to the simulator
    Actuators::getInstance().sendToSimulator();
#endif  // HILSimulation
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

            Actuators::getInstance().setServoAngle(AIRBRAKES_SERVO,
                                                   DPL_SERVO_RESET_POS);
            Actuators::getInstance().enableServo(AIRBRAKES_SERVO);

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
            Actuators::getInstance().setServoAngle(AIRBRAKES_SERVO,
                                                   ABK_SERVO_ROTATION);
            break;
        }
        case ABK_RESET:
        {
            Actuators::getInstance().setServoAngle(AIRBRAKES_SERVO, 0);
            break;
        }
        case FLIGHT_LIFTOFF_DETECTED:
        {
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
            abk.begin();

            return logStatus(AirBrakesControllerState::ACTIVE);
        }
        case FLIGHT_APOGEE_DETECTED:
        {
            return transition(&AirBrakesController::state_end);
        }
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
            abk.end();

            Actuators::getInstance().setServoAngle(AIRBRAKES_SERVO, 0);

            return logStatus(AirBrakesControllerState::END);
        }
    }
}

AirBrakesController::AirBrakesController()
    : FSM(&AirBrakesController::state_init),
      abk(
          []() {
              return TimedTrajectoryPoint{
                  NASController::getInstance().getNasState()};
          },
          TRAJECTORY_SET, AirBrakesControllerConfigs::ABK_CONFIG,
          [](float position) {
              Actuators::getInstance().setServo(ServosList::AIRBRAKES_SERVO,
                                                position);
          })
{
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

    Logger::getInstance().log(state);
}

void AirBrakesController::wiggleServo()
{
    for (int i = 0; i < 2; i++)
    {
        Actuators::getInstance().setServoAngle(AIRBRAKES_SERVO,
                                               ABK_SERVO_ROTATION);
        miosix::Thread::sleep(500);
        Actuators::getInstance().setServoAngle(AIRBRAKES_SERVO, 0);
        miosix::Thread::sleep(500);
    }
}

}  // namespace Main
