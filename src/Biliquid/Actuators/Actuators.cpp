/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <Biliquid/Configs/ActuatorsConfig.h>
#include <Biliquid/Debug.h>
#include <Biliquid/hwmapping.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

#include "ActuatorsData.h"
#include "ActuatorsMacros.h"

using namespace std::chrono;
using namespace Boardcore;
using namespace miosix;
using namespace Common;
using namespace Biliquid;

const Actuators::TimePoint Actuators::ValveClosed = TimePoint{};

void Actuators::ValveInfo::open(float position)
{
    direction =
        position >= currentPosition ? Direction::OPEN : Direction::CLOSE;

    // Account for the backstep amount, which will be applied later
    switch (direction)
    {
        case Direction::OPEN:
            position += Config::Servos::SERVO_BACKSTEP_AMOUNT;
            break;
        case Direction::CLOSE:
            position -= Config::Servos::SERVO_BACKSTEP_AMOUNT;
            break;
    }

    // Clamp the position to the [0, 1] range
    position = std::min(1.0f, std::max(0.0f, position));

    float delta        = std::abs(position - currentPosition);
    auto backstepDelay = milliseconds{static_cast<int>(
        Config::Servos::SERVO_FULL_RANGE_TIME.count() * delta)};

    currentPosition = position;
    backstepTs      = Clock::now() + backstepDelay;
}

void Actuators::ValveInfo::close()
{
    float delta        = currentPosition;
    auto backstepDelay = milliseconds{static_cast<int>(
        Config::Servos::SERVO_FULL_RANGE_TIME.count() * delta)};

    currentPosition = 0.0f;
    backstepTs      = Clock::now() + backstepDelay;
    direction       = Direction::CLOSE;
}

void Actuators::ValveInfo::backstep()
{
    switch (direction)
    {
        case Direction::OPEN:
            currentPosition -= Config::Servos::SERVO_BACKSTEP_AMOUNT;
            break;

        case Direction::CLOSE:
            currentPosition += Config::Servos::SERVO_BACKSTEP_AMOUNT;
            break;
    }

    // Clamp the position to the [0, 1] range
    currentPosition = std::min(1.0f, std::max(0.0f, currentPosition));

    move();
    backstepTs = ValveClosed;  // Reset backstep time
}

void Actuators::ValveInfo::move()
{
    PRINT_DEBUG("\tMoving valve {} to position {:05.3f} ({:05.3f} deg)\n",
                config.id, currentPosition, toDegrees(currentPosition));
    servo->setPosition(scalePosition(currentPosition));
}

float Actuators::ValveInfo::scalePosition(float position)
{
    position *= config.limit;
    if (config.flipped)
        position = 1.0f - position;

    return position;
}

float Actuators::ValveInfo::toDegrees(float position)
{
    return position * config.limit * 180.0f;  // Scale to degrees
}

Actuators::Actuators()
    : SignaledDeadlineTask(miosix::STACK_DEFAULT_FOR_PTHREAD,
                           miosix::PRIORITY_MAX - 1),
      valves{MAKE_SIMPLE_SERVO(MAIN_OX), MAKE_SIMPLE_SERVO(MAIN_FUEL)}
{
}

bool Actuators::isStarted() { return started; }

bool Actuators::start()
{
    if (!SignaledDeadlineTask::start())
    {
        LOG_ERR(logger, "Failed to start Actuators task");
        return false;
    }

    // Enable all servos
    for (auto& valve : valves)
    {
        valve.close();
        valve.move();  // Move to initial position
        valve.servo->enable();
    }

    // Set the initial position of the valves
    signalTask();

    started = true;
    return true;
}

bool Actuators::openValve(Valve valveId, float position)
{
    ValveInfo* valve = getValve(valveId);
    if (!valve)
        return false;

    {
        Lock<FastMutex> lock(valveMutex);
        valve->open(position);
    }

    signalTask();
    return true;
}

bool Actuators::closeValve(Valve valveId)
{
    ValveInfo* valve = getValve(valveId);
    if (!valve)
        return false;

    {
        Lock<FastMutex> lock(valveMutex);
        valve->close();
    }

    signalTask();
    return true;
}

void Actuators::closeAll()
{
    {
        Lock<FastMutex> lock(valveMutex);
        for (auto& valve : valves)
            valve.close();
    }

    signalTask();
}

Actuators::ValveInfo* Actuators::getValve(Valve valveId)
{
    switch (valveId)
    {
        case Valve::MAIN_OX:
            return &valves[0];
        case Valve::MAIN_FUEL:
            return &valves[1];

        default:
            return nullptr;
    }
}

SignaledDeadlineTask::TimePoint Actuators::nextTaskDeadline()
{
    Lock<FastMutex> lock(valveMutex);

    // Get the closest deadline from all valves
    return std::accumulate(valves.cbegin(), valves.cend(), TimePoint::max(),
                           [](TimePoint acc, const auto& valve)
                           {
                               return valve.backstepTs != ValveClosed
                                          ? std::min(acc, valve.backstepTs)
                                          : acc;
                           });
}

void Actuators::task()
{
    Lock<FastMutex> lock(valveMutex);

    auto currentTime = Clock::now();

    PRINT_DEBUG(
        "Actuators task @ {}ms:\n",
        duration_cast<milliseconds>(currentTime.time_since_epoch()).count());

    for (auto& valve : valves)
    {
        if (currentTime < valve.backstepTs)
        {
            valve.move();
        }
        else if (valve.backstepTs != ValveClosed)
        {
            // Backstep the valve a little to avoid strain
            valve.backstep();
        }
    }
}

template <>
struct fmt::formatter<Valve>
{
    auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const Valve& valve, FormatContext& ctx)
    {
        switch (valve)
        {
            case Valve::MAIN_OX:
                return fmt::format_to(ctx.out(), "MAIN_OX");
            case Valve::MAIN_FUEL:
                return fmt::format_to(ctx.out(), "MAIN_FUEL");
            default:
                return fmt::format_to(ctx.out(), "UNKNOWN_VALVE");
        }
    }
};
