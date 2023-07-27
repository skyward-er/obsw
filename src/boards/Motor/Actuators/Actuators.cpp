/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro, Alberto Nidasio
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

#include "Actuators.h"

#include <Motor/Configs/ActuatorsConfig.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>

#include "ActuatorsData.h"

using namespace miosix;
using namespace Boardcore;
using namespace Motor::ActuatorsConfig;

namespace Motor
{

Actuators::Actuators(TaskScheduler* sched) : scheduler(sched)
{
    servoMain =
        new Servo(SERVO_MAIN_TIMER, SERVO_MAIN_PWM_CH, MAX_PULSE, MIN_PULSE);
    servoVenting = new Servo(SERVO_VENTING_TIMER, SERVO_VENTING_PWM_CH,
                             MIN_PULSE, MAX_PULSE);

    // Set the default openings
    openings[ServosList::MAIN_VALVE]    = DEFAULT_MAIN_MAXIMUM_APERTURE;
    openings[ServosList::VENTING_VALVE] = DEFAULT_VENTING_MAXIMUM_APERTURE;

    // Set the default opening times
    openingTimes[ServosList::MAIN_VALVE]    = DEFAULT_MAIN_OPENING_TIME;
    openingTimes[ServosList::VENTING_VALVE] = DEFAULT_VENTING_OPENING_TIME;

    // Set the opening / closing events
    openingEvents[ServosList::MAIN_VALVE] =
        Common::Events::MOTOR_OPEN_FEED_VALVE;
    openingEvents[ServosList::VENTING_VALVE] =
        Common::Events::MOTOR_OPEN_VENTING_VALVE;

    closingEvents[ServosList::MAIN_VALVE] =
        Common::Events::MOTOR_CLOSE_FEED_VALVE;
    closingEvents[ServosList::VENTING_VALVE] =
        Common::Events::MOTOR_CLOSE_VENTING_VALVE;
}

bool Actuators::start()
{
    servoMain->enable();
    servoVenting->enable();

    uint8_t result = scheduler->addTask([=]() { checkTimings(); },
                                        SERVO_TIMINGS_CHECK_PERIOD);
    return result != 0;
}

bool Actuators::wiggleServo(ServosList servo)
{
    PauseKernelLock lock;
    // Get the choosen servo and nullptr in case not present
    Servo* requestedServo = getServo(servo);
    if (requestedServo != nullptr)
    {
        // Close all the servo
        closeAllServo();

        // Store the previous aperture time
        uint64_t time = openingTimes[servo];

        // Set a new one
        openingTimes[servo] = 1000;  // 1s

        // Toggle the servo
        toggleServo(servo);

        // Set the previous opening time
        openingTimes[servo] = time;

        return true;
    }

    return false;
}

Servo* Actuators::getServo(ServosList servo)
{
    switch (servo)
    {
        case MAIN_VALVE:
            return servoMain;
        case VENTING_VALVE:
            return servoVenting;
        default:
            return nullptr;
    }
}

float Actuators::getServoPosition(ServosList servo)
{
    // Get the choosen servo and nullptr in case not present
    Servo* requestedServo = getServo(servo);

    if (requestedServo == nullptr)
    {
        return -1;
    }

    return requestedServo->getPosition();
}

void Actuators::setServoPosition(ServosList servo, float position)
{
    // To lock the resources use a kernel lock
    PauseKernelLock lock;

    // Get the choosen servo and nullptr in case not present
    Servo* requestedServo = getServo(servo);

    if (!(requestedServo == nullptr))
    {
        requestedServo->setPosition(position);

        // Log the position
        ActuatorsData data;
        data.timestamp = TimestampTimer::getTimestamp();
        data.servoId   = servo;
        data.position  = position;
        Logger::getInstance().log(data);
    }
}

void Actuators::checkTimings()
{
    uint64_t currentTick = getTick();

    // Enter in protected zone where the timings should be checked and changed
    // and the servo should be positioned atomically over all the threads. A
    // kernel lock is adopted only due to it's performance but a mutex could be
    // easily adopted.
    {
        PauseKernelLock lock;

        for (uint8_t i = 0; i < ServosList::ServosList_ENUM_END; i++)
        {
            if (timings[i] > currentTick)
            {
                if (currentTick > setFlag[i] + SERVO_CONFIDENCE_TIME)
                {
                    setServoPosition(
                        static_cast<ServosList>(i),
                        openings[i] -
                            openings[i] *

                                SERVO_CONFIDENCE);  // 2% less than the
                                                    // actual aperture
                }
                else
                {
                    // Open the corresponding valve wrt the maximum opening
                    setServoPosition(static_cast<ServosList>(i), openings[i]);
                }
            }
            else
            {
                if (timings[i] != 0)
                {
                    timings[i] = 0;
                    setFlag[i] = currentTick;

                    {
                        RestartKernelLock l(lock);

                        // Publish the closing event
                        EventBroker::getInstance().post(
                            closingEvents[i], Common::Topics::TOPIC_MOTOR);
                    }
                }
                if (currentTick > setFlag[i] + SERVO_CONFIDENCE_TIME)
                {
                    setServoPosition(
                        static_cast<ServosList>(i),
                        openings[i] * SERVO_CONFIDENCE);  // 2% open
                }
                else
                {
                    setServoPosition(static_cast<ServosList>(i), 0);
                }
            }
        }
    }
}

void Actuators::toggleServo(ServosList servo)
{
    PauseKernelLock lock;

    if (getServo(servo) != nullptr)
    {
        // If the valve is already open
        if (timings[servo] > 0)
        {
            timings[servo] = 0;
            setFlag[servo] = getTick();

            {
                RestartKernelLock l(lock);

                // Publish the closing event
                EventBroker::getInstance().post(closingEvents[servo],
                                                Common::Topics::TOPIC_MOTOR);
            }
        }
        else
        {
            timings[servo] = getTick() + openingTimes[servo];
            setFlag[servo] = getTick();

            {
                RestartKernelLock l(lock);

                // Publish the opening event
                EventBroker::getInstance().post(openingEvents[servo],
                                                Common::Topics::TOPIC_MOTOR);
            }
        }
    }
}

void Actuators::openServoAtomic(ServosList servo, uint32_t time)
{
    PauseKernelLock lock;

    if (getServo(servo) != nullptr)
    {
        // If the valve is already open
        if (timings[servo] > 0)
        {
            timings[servo] = 0;
            setFlag[servo] = getTick();

            {
                RestartKernelLock l(lock);

                // Publish the closing event
                EventBroker::getInstance().post(closingEvents[servo],
                                                Common::Topics::TOPIC_MOTOR);
            }
        }
        else
        {
            timings[servo] = getTick() + time;
            setFlag[servo] = getTick();

            {
                RestartKernelLock l(lock);

                // Publish the opening event
                EventBroker::getInstance().post(openingEvents[servo],
                                                Common::Topics::TOPIC_MOTOR);
            }
        }
    }
}

void Actuators::closeAllServo()
{
    PauseKernelLock lock;

    for (uint8_t i = 0; i < ServosList::ServosList_ENUM_END; i++)
    {
        // Once the disconnect servo is open it shall never close
        if (timings[i] != 0 && i != ServosList::DISCONNECT_SERVO)
        {
            // Make the timings expire
            timings[i] = 0;
            setFlag[i] = getTick();
        }
    }
}

void Actuators::setMaximumAperture(ServosList servo, float aperture)
{
    PauseKernelLock lock;

    // Check if the servo exists
    if (getServo(servo) != nullptr)
    {
        // Check aperture
        if (aperture < 0)
        {
            aperture = 0;
        }
        else if (aperture > 1)
        {
            aperture = 1;
        }

        openings[servo] = aperture;
    }
}

void Actuators::setTiming(ServosList servo, uint32_t time)
{
    PauseKernelLock lock;

    // Check if the servo exists
    if (getServo(servo) != nullptr)
    {
        openingTimes[servo] = time;
    }
}

uint32_t Actuators::getTiming(ServosList servo)
{
    PauseKernelLock lock;

    if (getServo(servo) != nullptr)
    {
        return openingTimes[servo];
    }
    return 0;
}

}  // namespace Motor