/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

#include <RIGv2/Actuators/ActuatorsData.h>
#include <RIGv2/Configs/ActuatorsConfig.h>
#include <common/Events.h>
// TODO(davide.mor): Remove TimestampTimer
#include <drivers/timer/TimestampTimer.h>

using namespace Boardcore;
using namespace miosix;
using namespace Common;
using namespace RIGv2;

Actuators::Actuators(Boardcore::TaskScheduler &scheduler) : scheduler{scheduler}
{
    // Initialize servos
    infos[0].servo = std::make_unique<Servo>(
        MIOSIX_SERVOS_1_TIM, TimerUtils::Channel::MIOSIX_SERVOS_1_CHANNEL,
        Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE);
    infos[1].servo = std::make_unique<Servo>(
        MIOSIX_SERVOS_2_TIM, TimerUtils::Channel::MIOSIX_SERVOS_2_CHANNEL,
        Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE);
    infos[2].servo = std::make_unique<Servo>(
        MIOSIX_SERVOS_3_TIM, TimerUtils::Channel::MIOSIX_SERVOS_3_CHANNEL,
        Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE);
    infos[3].servo = std::make_unique<Servo>(
        MIOSIX_SERVOS_4_TIM, TimerUtils::Channel::MIOSIX_SERVOS_4_CHANNEL,
        Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE);
    // This servo is currently unusable, due to it sharing the same timer as
    // miosix, TIM5 infos[4].servo = std::make_unique<Servo>(
    //     MIOSIX_SERVOS_5_TIM, TimerUtils::Channel::MIOSIX_SERVOS_5_CHANNEL,
    //     Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE);
    infos[5].servo = std::make_unique<Servo>(
        MIOSIX_SERVOS_6_TIM, TimerUtils::Channel::MIOSIX_SERVOS_6_CHANNEL,
        Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE);
    infos[6].servo = std::make_unique<Servo>(
        MIOSIX_SERVOS_7_TIM, TimerUtils::Channel::MIOSIX_SERVOS_7_CHANNEL,
        Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE);
    // This servo is currently unusable, due to it sharing the same timer as
    // servo 1 infos[7].servo = std::make_unique<Servo>(
    //     MIOSIX_SERVOS_8_TIM, TimerUtils::Channel::MIOSIX_SERVOS_8_CHANNEL,
    //     Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE);
    infos[8].servo = std::make_unique<Servo>(
        MIOSIX_SERVOS_9_TIM, TimerUtils::Channel::MIOSIX_SERVOS_9_CHANNEL,
        Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE);
    infos[9].servo = std::make_unique<Servo>(
        MIOSIX_SERVOS_10_TIM, TimerUtils::Channel::MIOSIX_SERVOS_10_CHANNEL,
        Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE);

    ServoInfo *info;
    info               = getServo(ServosList::FILLING_VALVE);
    info->maxAperture  = Config::Servos::DEFAULT_FILLING_MAXIMUM_APERTURE;
    info->openingTime  = Config::Servos::DEFAULT_FILLING_OPENING_TIME;
    info->flipped      = Config::Servos::FILLING_FLIPPED;
    info->openingEvent = Common::Events::MOTOR_OPEN_FILLING_VALVE;
    info->closingEvent = Common::Events::MOTOR_CLOSE_FILLING_VALVE;

    info               = getServo(ServosList::RELEASE_VALVE);
    info->maxAperture  = Config::Servos::DEFAULT_RELEASE_MAXIMUM_APERTURE;
    info->openingTime  = Config::Servos::DEFAULT_RELEASE_OPENING_TIME;
    info->flipped      = Config::Servos::RELEASE_FLIPPED;
    info->openingEvent = Common::Events::MOTOR_OPEN_RELEASE_VALVE;
    info->closingEvent = Common::Events::MOTOR_CLOSE_RELEASE_VALVE;

    info               = getServo(ServosList::DISCONNECT_SERVO);
    info->maxAperture  = Config::Servos::DEFAULT_DISCONNECT_MAXIMUM_APERTURE;
    info->openingTime  = Config::Servos::DEFAULT_DISCONNECT_OPENING_TIME;
    info->flipped      = Config::Servos::DISCONNECT_FLIPPED;
    info->openingEvent = Common::Events::MOTOR_DISCONNECT;

    info               = getServo(ServosList::MAIN_VALVE);
    info->maxAperture  = Config::Servos::DEFAULT_MAIN_MAXIMUM_APERTURE;
    info->openingTime  = Config::Servos::DEFAULT_MAIN_OPENING_TIME;
    info->flipped      = Config::Servos::MAIN_FLIPPED;
    info->openingEvent = Common::Events::MOTOR_OPEN_FEED_VALVE;
    info->closingEvent = Common::Events::MOTOR_CLOSE_FEED_VALVE;

    // This servo is not yet enabled
    // info               = getServo(ServosList::VENTING_VALVE);
    // info->maxAperture  = Config::Servos::DEFAULT_VENTING_MAXIMUM_APERTURE;
    // info->openingTime  = Config::Servos::DEFAULT_VENTING_OPENING_TIME;
    // info->flipped      = true;
    // info->openingEvent = Common::Events::MOTOR_OPEN_VENTING_VALVE;
    // info->closingEvent = Common::Events::MOTOR_CLOSE_VENTING_VALVE;
}

bool Actuators::start()
{
    infos[0].servo->enable();
    infos[1].servo->enable();
    infos[2].servo->enable();
    infos[3].servo->enable();
    // infos[4].servo->enable();
    infos[5].servo->enable();
    infos[6].servo->enable();
    // infos[7].servo->enable();
    infos[8].servo->enable();
    infos[9].servo->enable();

    updatePositionTaskId =
        scheduler.addTask([this]() { updatePositionsTask(); },
                          Config::Servos::SERVO_TIMINGS_CHECK_PERIOD);

    if (updatePositionTaskId == 0)
    {
        LOG_ERR(logger, "Failed to add updatePositionsTask");
        return false;
    }

    return true;
}

bool Actuators::wiggleServo(ServosList servo)
{
    // Wiggle means open the servo for 1s
    return openServoAtomic(servo, 1000);
}

bool Actuators::toggleServo(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo *info = getServo(servo);
    if (info == nullptr)
    {
        return false;
    }

    if (info->closeTs == 0)
    {
        // The servo is closed, open it
        openServoInner(info, info->openingTime);
    }
    else
    {
        // The servo is open, close it
        closeServoInner(info);
    }

    return true;
}

bool Actuators::closeServo(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo *info = getServo(servo);
    if (info == nullptr)
    {
        return false;
    }

    closeServoInner(info);
    return true;
}

bool Actuators::openServoAtomic(ServosList servo, uint64_t time)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo *info = getServo(servo);
    if (info == nullptr)
    {
        return false;
    }

    openServoInner(info, time);
    return true;
}

bool Actuators::setMaxAperture(ServosList servo, float aperture)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo *info = getServo(servo);
    if (info == nullptr)
    {
        return false;
    }

    info->maxAperture = aperture;
    return true;
}

bool Actuators::setOpeningTime(ServosList servo, uint64_t time)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo *info = getServo(servo);
    if (info == nullptr)
    {
        return false;
    }

    info->openingTime = time;
    return true;
}

bool Actuators::isServoOpen(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo *info = getServo(servo);
    if (info == nullptr)
    {
        return false;
    }

    return info->servo->getPosition() > Config::Servos::SERVO_OPEN_THRESHOLD;
}

void Actuators::openServoInner(ServoInfo *info, uint64_t time)
{
    long long currentTime = getTime();

    info->openedTs = currentTime;
    info->closeTs  = currentTime + (time * Constants::NS_IN_MS);

    // TODO(davide.mor): Dispatch the open event
}

void Actuators::closeServoInner(ServoInfo *info)
{
    info->closeTs = 0;

    // TODO(davide.mor): Dispatch the close event
}

Actuators::ServoInfo *Actuators::getServo(ServosList servo)
{
    switch (servo)
    {
        case FILLING_VALVE:
            return &infos[3];
        case RELEASE_VALVE:
            return &infos[2];
        case DISCONNECT_SERVO:
            return &infos[1];
        case MAIN_VALVE:
            return &infos[0];
            // TODO(davide.mor): Decide this servo
            // case VENTING_VALVE:
            //     return &infos[8];

        default:
            // Oh FUCK
            LOG_ERR(logger, "Invalid servo requested");
            return nullptr;
    }
}

void Actuators::unsafeSetServoPosition(uint8_t idx, float position)
{
    // Invert the position if the servo is flipped
    if (infos[idx].flipped)
    {
        infos[idx].servo->setPosition(1.0 - position);
    }
    else
    {
        infos[idx].servo->setPosition(position);
    }

    // Log the update
    ActuatorsData data;
    data.timestamp = TimestampTimer::getTimestamp();
    data.servoIdx  = idx;
    data.position  = position;
    sdLogger.log(data);
}

void Actuators::updatePositionsTask()
{
    Lock<FastMutex> lock(infosMutex);

    long long currentTime = getTime();

    // Iterate over all servos
    for (uint8_t idx = 0; idx < 10; idx++)
    {
        if (currentTime < infos[idx].closeTs)
        {
            // The valve should be open
            if (currentTime <
                infos[idx].openedTs + (Config::Servos::SERVO_CONFIDENCE_TIME *
                                       Constants::NS_IN_MS))
            {
                // We should open the valve all the way
                unsafeSetServoPosition(idx, infos[idx].maxAperture);
            }
            else
            {
                // Time to wiggle the valve a little
                unsafeSetServoPosition(
                    idx, infos[idx].maxAperture *
                             (1.0 - Config::Servos::SERVO_CONFIDENCE));
            }
        }
        else
        {
            // Ok the valve should be closed
            if (infos[idx].closeTs != 0)
            {
                // The valve JUST closed, notify everybody
                infos[idx].closeTs = 0;

                // TODO(davide.mor): Dispatch the close event
            }

            if (currentTime <
                infos[idx].closeTs + (Config::Servos::SERVO_CONFIDENCE_TIME *
                                      Constants::NS_IN_MS))
            {
                // We should close the valve all the way
                unsafeSetServoPosition(idx, 0.0);
            }
            else
            {
                // Time to wiggle the valve a little
                unsafeSetServoPosition(idx, Config::Servos::SERVO_CONFIDENCE);
            }
        }
    }
}