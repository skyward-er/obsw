/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro
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
#include <RIG/Actuators/Actuators.h>
#include <RIG/Actuators/ActuatorsData.h>
#include <RIG/CanHandler/CanHandler.h>
#include <RIG/Configs/ActuatorsConfig.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;

namespace RIG
{
Actuators::Actuators(TaskScheduler* sched) : scheduler(sched)
{
    servo1 =
        new Servo(Config::Servos::SERVO1_TIMER, Config::Servos::SERVO1_PWM_CH,
                  Config::Servos::MAX_PULSE, Config::Servos::MIN_PULSE);
    servo2 =
        new Servo(Config::Servos::SERVO2_TIMER, Config::Servos::SERVO2_PWM_CH,
                  Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE);
    servo3 =
        new Servo(Config::Servos::SERVO3_TIMER, Config::Servos::SERVO3_PWM_CH,
                  Config::Servos::MAX_PULSE, Config::Servos::MIN_PULSE);
    servo4 =
        new Servo(Config::Servos::SERVO4_TIMER, Config::Servos::SERVO4_PWM_CH,
                  Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE);
    servo5 =
        new Servo(Config::Servos::SERVO5_TIMER, Config::Servos::SERVO5_PWM_CH,
                  Config::Servos::MAX_PULSE, Config::Servos::MIN_PULSE);

    // Set the default openings
    openings[ServosList::FILLING_VALVE] =
        Config::Servos::DEFAULT_FILLING_MAXIMUM_APERTURE;
    openings[ServosList::MAIN_VALVE] =
        Config::Servos::DEFAULT_MAIN_MAXIMUM_APERTURE;
    openings[ServosList::RELEASE_VALVE] =
        Config::Servos::DEFAULT_RELEASE_MAXIMUM_APERTURE;
    openings[ServosList::VENTING_VALVE] =
        Config::Servos::DEFAULT_VENTING_MAXIMUM_APERTURE;
    openings[ServosList::DISCONNECT_SERVO] =
        Config::Servos::DEFAULT_DISCONNECT_MAXIMUM_APERTURE;

    // Set the default opening times
    openingTimes[ServosList::FILLING_VALVE] =
        Config::Servos::DEFAULT_FILLING_OPENING_TIME;
    openingTimes[ServosList::MAIN_VALVE] =
        Config::Servos::DEFAULT_MAIN_OPENING_TIME;
    openingTimes[ServosList::RELEASE_VALVE] =
        Config::Servos::DEFAULT_RELEASE_OPENING_TIME;
    openingTimes[ServosList::VENTING_VALVE] =
        Config::Servos::DEFAULT_VENTING_OPENING_TIME;
    openingTimes[ServosList::DISCONNECT_SERVO] =
        Config::Servos::DEFAULT_DISCONNECT_OPENING_TIME;

    // Set the opening / closing events
    openingEvents[ServosList::FILLING_VALVE] =
        Common::Events::MOTOR_OPEN_FILLING_VALVE;
    openingEvents[ServosList::MAIN_VALVE] =
        Common::Events::MOTOR_OPEN_FEED_VALVE;
    openingEvents[ServosList::RELEASE_VALVE] =
        Common::Events::MOTOR_OPEN_RELEASE_VALVE;
    openingEvents[ServosList::VENTING_VALVE] =
        Common::Events::MOTOR_OPEN_VENTING_VALVE;
    openingEvents[ServosList::DISCONNECT_SERVO] =
        Common::Events::MOTOR_DISCONNECT;

    closingEvents[ServosList::FILLING_VALVE] =
        Common::Events::MOTOR_CLOSE_FILLING_VALVE;
    closingEvents[ServosList::MAIN_VALVE] =
        Common::Events::MOTOR_CLOSE_FEED_VALVE;
    closingEvents[ServosList::RELEASE_VALVE] =
        Common::Events::MOTOR_CLOSE_RELEASE_VALVE;
    closingEvents[ServosList::VENTING_VALVE] =
        Common::Events::MOTOR_CLOSE_VENTING_VALVE;
}

bool Actuators::start()
{
    servo1->enable();
    servo2->enable();
    servo3->enable();
    servo4->enable();
    servo5->enable();

    uint8_t result = scheduler->addTask(
        [=]() { checkTimings(); }, Config::Servos::SERVO_TIMINGS_CHECK_PERIOD);
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
            return servo4;
        case VENTING_VALVE:
            return servo5;
        case RELEASE_VALVE:
            return servo2;
        case FILLING_VALVE:
            return servo1;
        case DISCONNECT_SERVO:
            return servo3;
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
    uint64_t currentTime = getTime() / Constants::NS_IN_MS; // [ms]

    // Enter in protected zone where the timings should be checked and changed
    // and the servo should be positioned atomically over all the threads. A
    // kernel lock is adopted only due to it's performance but a mutex could be
    // easily adopted.
    {
        PauseKernelLock lock;

        for (uint8_t i = 0; i < ServosList::ServosList_ENUM_END; i++)
        {
            if (timings[i] > currentTime)
            {
                if (currentTime >
                    setFlag[i] + Config::Servos::SERVO_CONFIDENCE_TIME)
                {
                    setServoPosition(
                        static_cast<ServosList>(i),
                        openings[i] -
                            openings[i] *
                                Config::Servos::
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
                    setFlag[i] = currentTime;

                    {
                        RestartKernelLock l(lock);

                        // Publish the closing event
                        EventBroker::getInstance().post(
                            closingEvents[i], Common::Topics::TOPIC_MOTOR);
                    }
                }
                if (currentTime >
                    setFlag[i] + Config::Servos::SERVO_CONFIDENCE_TIME)
                {
                    setServoPosition(
                        static_cast<ServosList>(i),
                        openings[i] *
                            Config::Servos::SERVO_CONFIDENCE);  // 2% open
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
            setFlag[servo] = getTime() / Constants::NS_IN_MS;

            {
                RestartKernelLock l(lock);

                // Publish the closing event
                EventBroker::getInstance().post(closingEvents[servo],
                                                Common::Topics::TOPIC_MOTOR);

                // Publish the command also into the CAN bus
                ModuleManager::getInstance()
                    .get<CanHandler>()
                    ->sendCanServoCommand(servo, 0, 0);
            }
        }
        else
        {
            long long timeMs = getTime() / Constants::NS_IN_MS;
            timings[servo] = timeMs + openingTimes[servo];
            setFlag[servo] = timeMs;

            {
                RestartKernelLock l(lock);

                // Publish the opening event
                EventBroker::getInstance().post(openingEvents[servo],
                                                Common::Topics::TOPIC_MOTOR);

                // Publish the command also into the CAN bus
                ModuleManager::getInstance()
                    .get<CanHandler>()
                    ->sendCanServoCommand(servo, 1, openingTimes[servo]);
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
            setFlag[servo] = getTime() / Constants::NS_IN_MS;;

            {
                RestartKernelLock l(lock);

                // Publish the closing event
                EventBroker::getInstance().post(closingEvents[servo],
                                                Common::Topics::TOPIC_MOTOR);

                // Publish the command also into the CAN bus
                ModuleManager::getInstance()
                    .get<CanHandler>()
                    ->sendCanServoCommand(servo, 0, 0);
            }
        }
        else
        {
            long long timeMs = getTime() / Constants::NS_IN_MS;
            timings[servo] = timeMs + time;
            setFlag[servo] = timeMs;

            {
                RestartKernelLock l(lock);

                // Publish the opening event
                EventBroker::getInstance().post(openingEvents[servo],
                                                Common::Topics::TOPIC_MOTOR);

                // Publish the command also into the CAN bus
                ModuleManager::getInstance()
                    .get<CanHandler>()
                    ->sendCanServoCommand(servo, 1, time);
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
            setFlag[i] = getTime() / Constants::NS_IN_MS;;

            // Publish the command also into the CAN bus
            ModuleManager::getInstance().get<CanHandler>()->sendCanServoCommand(
                static_cast<ServosList>(i), 0, 0);
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

}  // namespace RIG