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

#include "Actuators.h"

#include <Motor/Actuators/ActuatorsData.h>
#include <Motor/Configs/ActuatorsConfig.h>
#include <interfaces-impl/hwmapping.h>

using namespace miosix;
using namespace Boardcore;
using namespace Motor;

void Actuators::ServoInfo::openServoWithTime(uint32_t time)
{
    long long currentTime = getTime();

    closeTs      = currentTime + (time * Constants::NS_IN_MS);
    lastActionTs = currentTime;
}

void Actuators::ServoInfo::closeServo()
{
    closeTs      = 0;
    lastActionTs = getTime();
}

void Actuators::ServoInfo::unsafeSetServoPosition(float position)
{
    // Check that the servo is actually there, just to be safe
    if (!servo)
    {
        return;
    }

    position *= limit;
    if (flipped)
    {
        position = 1.0f - position;
    }

    servo->setPosition(position);
}

float Actuators::ServoInfo::getServoPosition()
{
    // Check that the servo is actually there, just to be safe
    if (!servo)
    {
        return 0.0f;
    }

    float position = servo->getPosition();
    if (flipped)
    {
        position = 1.0f - position;
    }

    position /= limit;
    return position;
}

Actuators::Actuators()
{
    infos[0].servo = std::make_unique<Servo>(
        MIOSIX_SERVOS_1_TIM, TimerUtils::Channel::MIOSIX_SERVOS_1_CHANNEL,
        Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE,
        Config::Servos::FREQUENCY);
    infos[1].servo = std::make_unique<Servo>(
        MIOSIX_SERVOS_2_TIM, TimerUtils::Channel::MIOSIX_SERVOS_2_CHANNEL,
        Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE,
        Config::Servos::FREQUENCY);

    ServoInfo *info;
    info          = getServo(ServosList::MAIN_VALVE);
    info->limit   = Config::Servos::MAIN_LIMIT;
    info->flipped = Config::Servos::MAIN_FLIPPED;
    info->unsafeSetServoPosition(0.0f);

    info          = getServo(ServosList::VENTING_VALVE);
    info->limit   = Config::Servos::VENTING_LIMIT;
    info->flipped = Config::Servos::VENTING_FLIPPED;
    info->unsafeSetServoPosition(0.0f);
}

bool Actuators::start()
{
    TaskScheduler &scheduler =
        getModule<BoardScheduler>()->getActuatorsScheduler();

    infos[0].servo->enable();
    infos[1].servo->enable();

    uint8_t result =
        scheduler.addTask([this]() { updatePositionsTask(); },
                          Config::Servos::SERVO_TIMINGS_CHECK_PERIOD);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add updatePositionsTask");
        return false;
    }

    return true;
}

bool Actuators::openServoWithTime(ServosList servo, uint32_t time)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo *info = getServo(servo);
    if (info == nullptr)
    {
        return false;
    }

    info->openServoWithTime(time);
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

    info->closeServo();
    return true;
}

float Actuators::getServoPosition(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo *info = getServo(servo);
    if (info == nullptr)
    {
        return 0.0f;
    }

    return info->getServoPosition();
}

bool Actuators::isServoOpen(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo *info = getServo(servo);
    if (info == nullptr)
    {
        return false;
    }

    return info->closeTs != 0;
}

Actuators::ServoInfo *Actuators::getServo(ServosList servo)
{
    switch (servo)
    {
        case VENTING_VALVE:
            return &infos[0];
        case MAIN_VALVE:
            return &infos[1];

        default:
            // Oh FUCK
            LOG_ERR(logger, "Invalid servo requested");
            return nullptr;
    }
}

void Actuators::unsafeSetServoPosition(uint8_t idx, float position)
{
    infos[idx].unsafeSetServoPosition(position);

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
    for (uint8_t idx = 0; idx < 2; idx++)
    {
        if (currentTime < infos[idx].closeTs)
        {
            // The valve should be open
            if (currentTime < infos[idx].lastActionTs +
                                  (Config::Servos::SERVO_CONFIDENCE_TIME *
                                   Constants::NS_IN_MS))
            {
                // We should open the valve all the way
                unsafeSetServoPosition(idx, 1.0f);
            }
            else
            {
                // Time to wiggle the valve a little
                unsafeSetServoPosition(idx,
                                       1.0 - Config::Servos::SERVO_CONFIDENCE);
            }
        }
        else
        {
            // Ok the valve should be closed
            if (infos[idx].closeTs != 0)
            {
                // Perform the servo closing
                infos[idx].closeServo();
            }

            if (currentTime < infos[idx].lastActionTs +
                                  (Config::Servos::SERVO_CONFIDENCE_TIME *
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