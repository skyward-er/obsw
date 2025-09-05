/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto
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

#include <chrono>

using namespace std::chrono;
using namespace miosix;
using namespace Boardcore;
using namespace Motor;

const Actuators::TimePoint Actuators::ValveClosed = TimePoint{};

void Actuators::ServoInfo::openServoWithTime(uint32_t time)
{
    auto currentTime = Clock::now();

    closeTs    = currentTime + milliseconds{time};
    backstepTs = currentTime + Config::Servos::SERVO_BACKSTEP_DELAY;
}

void Actuators::ServoInfo::closeServo()
{
    closeTs    = ValveClosed;
    backstepTs = Clock::now() + Config::Servos::SERVO_BACKSTEP_DELAY;
}

void Actuators::ServoInfo::unsafeSetServoPosition(float position)
{
    // Check that the servo is actually there, just to be safe
    if (!servo)
        return;

    position *= limit;
    if (flipped)
        position = 1.0f - position;

    servo->setPosition(position);
}

float Actuators::ServoInfo::getServoPosition()
{
    // Check that the servo is actually there, just to be safe
    if (!servo)
        return 0.0f;

    float position = servo->getPosition();
    if (flipped)
        position = 1.0f - position;

    position /= limit;
    return position;
}

Actuators::Actuators()
    : SignaledDeadlineTask(miosix::STACK_DEFAULT_FOR_PTHREAD,
                           BoardScheduler::actuatorsPriority())
{
    {
        ServoInfo* info = getServo(ServosList::OX_VENTING_VALVE);
        info->servo     = std::make_unique<Servo>(
            MIOSIX_SERVOS_0_TIM, TimerUtils::Channel::MIOSIX_SERVOS_0_CHANNEL,
            Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE,
            Config::Servos::FREQUENCY);

        info->limit   = Config::Servos::OX_VENTING_LIMIT;
        info->flipped = Config::Servos::OX_VENTING_FLIPPED;
        info->unsafeSetServoPosition(0.0f);
    }
    {
        ServoInfo* info = getServo(ServosList::MAIN_VALVE);
        info->servo     = std::make_unique<Servo>(
            MIOSIX_SERVOS_5_TIM, TimerUtils::Channel::MIOSIX_SERVOS_5_CHANNEL,
            Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE,
            Config::Servos::FREQUENCY);

        info->limit   = Config::Servos::MAIN_LIMIT;
        info->flipped = Config::Servos::MAIN_FLIPPED;
        info->unsafeSetServoPosition(0.0f);
    }
    {
        ServoInfo* info = getServo(ServosList::NITROGEN_VALVE);
        info->servo     = std::make_unique<Servo>(
            MIOSIX_SERVOS_4_TIM, TimerUtils::Channel::MIOSIX_SERVOS_4_CHANNEL,
            Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE,
            Config::Servos::FREQUENCY);

        info->limit   = Config::Servos::NITROGEN_LIMIT;
        info->flipped = Config::Servos::NITROGEN_FLIPPED;
        info->unsafeSetServoPosition(0.0f);
    }
    {
        ServoInfo* info = getServo(ServosList::N2_QUENCHING_VALVE);
        info->servo     = std::make_unique<Servo>(
            MIOSIX_SERVOS_3_TIM, TimerUtils::Channel::MIOSIX_SERVOS_3_CHANNEL,
            Config::Servos::SMALL_MIN_PULSE, Config::Servos::SMALL_MAX_PULSE,
            Config::Servos::FREQUENCY);

        info->limit   = Config::Servos::N2_QUENCHING_LIMIT;
        info->flipped = Config::Servos::N2_QUENCHING_FLIPPED;
        info->unsafeSetServoPosition(0.0f);
    }
}

bool Actuators::start()
{
    // Enable all servos and close them to force a backstep
    for (ServoInfo& info : infos)
    {
        info.servo->enable();
        info.closeServo();
    }

    // Reset the safety venting timestamp
    safetyVentingTs = Clock::now() + Config::Servos::SAFETY_VENTING_TIMEOUT;

    if (!SignaledDeadlineTask::start())
    {
        LOG_ERR(logger, "Failed to start Actuators task");
        return false;
    }

    return true;
}

bool Actuators::openServoWithTime(ServosList servo, uint32_t time)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    safetyVentingTs = Clock::now() + Config::Servos::SAFETY_VENTING_TIMEOUT;
    info->openServoWithTime(time);
    signalTask();
    return true;
}

bool Actuators::closeServo(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    safetyVentingTs = Clock::now() + Config::Servos::SAFETY_VENTING_TIMEOUT;
    info->closeServo();
    signalTask();
    return true;
}

float Actuators::getServoPosition(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo* info = getServo(servo);
    if (info == nullptr)
        return 0.0f;

    return info->getServoPosition();
}

bool Actuators::isServoOpen(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    return info->closeTs != ValveClosed;
}

Actuators::ServoInfo* Actuators::getServo(ServosList servo)
{
    switch (servo)
    {
        case OX_VENTING_VALVE:
            return &infos[0];  // Servo 0
        case MAIN_VALVE:
            return &infos[1];  // Servo 5
        case NITROGEN_VALVE:
            return &infos[2];  // Servo 4
        case N2_QUENCHING_VALVE:
            return &infos[3];  // Servo 3

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

SignaledDeadlineTask::TimePoint Actuators::nextTaskDeadline()
{
    Lock<FastMutex> lock(infosMutex);

    // Start with the maximum value
    auto nextDeadline = TimePoint::max();

    // Get the closest deadline from all valves
    for (auto& info : infos)
    {
        if (info.closeTs != ValveClosed)
            nextDeadline = std::min(nextDeadline, info.closeTs);

        if (info.backstepTs != ValveClosed)
            nextDeadline = std::min(nextDeadline, info.backstepTs);
    }

    nextDeadline = std::min(nextDeadline, safetyVentingTs);

    return nextDeadline;
}

void Actuators::task()
{
    Lock<FastMutex> lock(infosMutex);

    auto currentTime = Clock::now();

    // Iterate over all servos
    for (uint8_t idx = 0; idx < infos.size(); idx++)
    {
        if (currentTime < infos[idx].closeTs)
        {
            // The valve should be open
            if (currentTime < infos[idx].backstepTs)
            {
                // We should open the valve all the way
                unsafeSetServoPosition(idx, 1.0f);
            }
            else
            {
                // Backstep the valve a little to avoid strain
                unsafeSetServoPosition(
                    idx, 1.0 - Config::Servos::SERVO_BACKSTEP_AMOUNT);

                infos[idx].backstepTs = ValveClosed;  // Reset backstep time
            }
        }
        else
        {
            // Ok the valve should be closed
            if (infos[idx].closeTs != ValveClosed)
            {
                // Perform the servo closing
                infos[idx].closeServo();
            }

            if (currentTime < infos[idx].backstepTs)
            {
                // We should close the valve all the way
                unsafeSetServoPosition(idx, 0.0);
            }
            else
            {
                // Backstep the valve a little to avoid strain
                unsafeSetServoPosition(idx,
                                       Config::Servos::SERVO_BACKSTEP_AMOUNT);

                infos[idx].backstepTs = ValveClosed;  // Reset backstep time
            }
        }
    }

    // Check if we reached the inactivity timeout and should vent
    if (currentTime >= safetyVentingTs)
    {
        getServo(ServosList::OX_VENTING_VALVE)
            ->openServoWithTime(
                milliseconds{Config::Servos::SAFETY_VENTING_DURATION}.count());

        getServo(ServosList::NITROGEN_VALVE)
            ->openServoWithTime(
                milliseconds{Config::Servos::SAFETY_VENTING_DURATION}.count());

        // Reset the safety venting timestamp
        safetyVentingTs = currentTime + Config::Servos::SAFETY_VENTING_TIMEOUT;
    }
}

