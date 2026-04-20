/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto, Riccardo Sironi, Pietro Bortolus
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
#include <Motor/Actuators/ActuatorsMacros.h>
#include <Motor/Configs/ActuatorsConfig.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>

#include <chrono>

using namespace std::chrono;
using namespace miosix;
using namespace Boardcore;
using namespace Common;
using namespace Motor;

const Actuators::TimePoint Actuators::noActionNeeded = TimePoint{};

bool Actuators::ValveInfo::isValveOpen() { return valve->getPosition(); }

void Actuators::ValveInfo::backstep()
{
    valve->backstep();
    backstepTs = noActionNeeded;  // Reset backstep time
}

void Actuators::ValveInfo::openValve()
{
    valve->currentPosition = valve->getDefaultMaxAperture();
    valve->direction       = Valve::Direction::OPEN;
    backstepTs =
        Clock::now() + milliseconds{Config::Servos::SERVO_BACKSTEP_DELAY};

    resetAnimation();

    valve->setPosition(valve->currentPosition);

    const uint8_t openingEvent = valve->getOpeningEvent();
    if (openingEvent != 0)
        EventBroker::getInstance().post(openingEvent, TOPIC_MOTOR);
}

void Actuators::ValveInfo::closeValve()
{
    valve->currentPosition = 0.0f;
    valve->direction       = Valve::Direction::CLOSE;
    closeTs                = noActionNeeded;
    backstepTs =
        Clock::now() + milliseconds{Config::Servos::SERVO_BACKSTEP_DELAY};

    resetAnimation();

    valve->setPosition(valve->currentPosition);

    const uint8_t closingEvent = valve->getClosingEvent();
    if (closingEvent != 0)
        EventBroker::getInstance().post(closingEvent, TOPIC_MOTOR);
}

void Actuators::ManualValveInfo::animateValve(float position, uint32_t time)
{
    // Nothing to animate
    if (valve->currentPosition == position)
        return;

    valve->direction = position >= valve->currentPosition
                           ? Valve::Direction::OPEN
                           : Valve::Direction::CLOSE;

    // Account for the backstep amount, which will be applied later
    switch (valve->direction)
    {
        case Valve::Direction::OPEN:
            position += Config::Servos::SERVO_BACKSTEP_AMOUNT;
            break;
        case Valve::Direction::CLOSE:
            position -= Config::Servos::SERVO_BACKSTEP_AMOUNT;
            break;
    }

    // Clamp the position to the [0, 1] range
    position = std::min(1.0f, std::max(0.0f, position));

    float delta        = position - valve->currentPosition;
    auto backstepDelay = milliseconds{static_cast<int>(
        Config::Servos::SERVO_BACKSTEP_DELAY.count() * std::abs(delta))};

    auto currentTime = Clock::now();
    updateTs         = currentTime;  // Start the animation immediately
    backstepTs       = currentTime + nanoseconds{msToNs(time)} + backstepDelay;
    closeTs = noActionNeeded;  // The animate function does not close the servo
                               // automatically

    // we add one step since the first one will be taken immediately after
    // calling this function
    stepCount  = time / Config::Servos::ANIMATION_UPDATE_PERIOD.count() + 1;
    stepAmount = delta / stepCount;
}

void Actuators::ManualValveInfo::advanceAnimation()
{
    // Update the servo position
    valve->currentPosition += stepAmount;

    // Clamp position
    valve->currentPosition =
        std::min(1.0f, std::max(0.0f, valve->currentPosition));

    stepCount--;
    if (stepCount == 0)
    {
        // The animation is done, reset updateTs
        updateTs = noActionNeeded;
    }
    else
    {
        // The animation is not done, schedule the next update
        updateTs = Clock::now() + Config::Servos::ANIMATION_UPDATE_PERIOD;
    }

    valve->setPosition(valve->currentPosition);
}

void Actuators::ManualValveInfo::resetAnimation()
{
    stepCount  = 0;
    stepAmount = 0.0f;
    updateTs   = noActionNeeded;
}

Actuators::Actuators()
    : SignaledDeadlineTask(miosix::STACK_DEFAULT_FOR_PTHREAD,
                           BoardScheduler::actuatorsPriority()),
      spark(std::make_unique<SparkPlug>(
          MIOSIX_IGNITER_TIM, (uint16_t)50,
          TimerUtils::Channel::MIOSIX_IGNITER_CHANNEL))
{
}

bool Actuators::isStarted() { return started; }

bool Actuators::start()
{
    initializeValves();

    // Enable all servos and close them to force a backstep
    for (auto& info : valveInfos)
    {
        info.valve->enable();
        info.closeValve();
        info.backstepTs = Clock::now() + milliseconds{2000};
    }

    for (auto& info : manualValveInfos)
    {
        info.valve->enable();
        info.closeValve();
        info.backstepTs = Clock::now() + milliseconds{2000};
    }

    // Reset the safety venting timestamp
    safetyVentingTs = Clock::now() + Config::Servos::SAFETY_VENTING_TIMEOUT;

    spark->enable();

    if (!SignaledDeadlineTask::start())
    {
        LOG_ERR(logger, "Failed to start Actuators task");
        return false;
    }

    signalTask();
    started = true;
    return true;
}

void Actuators::initializeValves()
{
    valveInfos.push_back(MAKE_SERVO_VALVE(OX_VEN));
    valveInfos.push_back(MAKE_SERVO_VALVE(FUEL_VEN));

    // "Manual" servo valves, instead of only being fully open or closed, can be
    // moved to any position in the range [0, 1].
    manualValveInfos.push_back(MAKE_MANUAL_SERVO_VALVE(MAIN_OX));
    manualValveInfos.push_back(MAKE_MANUAL_SERVO_VALVE(MAIN_FUEL));
    manualValveInfos.push_back(MAKE_MANUAL_SERVO_VALVE(PRZ_OX));
    manualValveInfos.push_back(MAKE_MANUAL_SERVO_VALVE(PRZ_FUEL));

    // Solenoid valves connected directly to the micro
    valveInfos.push_back(
        MAKE_SOLENOID_VALVE(IGN_OX, actuators::oxSolenoid::getPin()));
    valveInfos.push_back(
        MAKE_SOLENOID_VALVE(IGN_FUEL, actuators::fuelSolenoid::getPin()));
}
bool Actuators::wiggleValve(ServosList servo)
{
    // Wiggle means open the servo for 1s
    return openValveWithTime(servo, 1000);
}

bool Actuators::toggleValve(ServosList servo)
{
    ValveInfo* info = getValve(servo);
    if (info == nullptr)
        return false;

    if (info->closeTs == noActionNeeded)
    {
        // The servo is closed, open it
        openValve(servo);
    }
    else
    {
        // The servo is open, close it
        closeValve(servo);
    }

    return true;
}

bool Actuators::openValve(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ValveInfo* info = getValve(servo);
    if (info == nullptr)
        return false;

    uint32_t time = getServoOpeningTime(servo);

    getModule<CanHandler>()->sendServoOpenCommand(servo, time);

    info->closeTs = Clock::now() + nanoseconds{msToNs(time)};

    signalTask();
    return true;
}

bool Actuators::openValveWithTime(ServosList servo, uint32_t time)
{
    Lock<FastMutex> lock(infosMutex);
    ValveInfo* info = getValve(servo);
    if (info == nullptr)
        return false;

    getModule<CanHandler>()->sendServoOpenCommand(servo, time);

    // tell the task to open this valve
    info->closeTs = Clock::now() + nanoseconds{msToNs(time)};

    signalTask();
    return true;
}

bool Actuators::closeValve(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ValveInfo* info = getValve(servo);
    if (info == nullptr)
        return false;

    getModule<CanHandler>()->sendServoCloseCommand(servo);

    // tell the task to close this valve
    info->closeTs = Clock::now();

    signalTask();
    return true;
}

bool Actuators::moveValve(ServosList servo, float position)
{
    Lock<FastMutex> lock(infosMutex);
    ManualValveInfo* info = getManualValve(servo);
    if (info == nullptr)
        return false;

    // moveValve is the same as doing an animation with time = 0
    info->animateValve(position, 0);

    signalTask();
    return true;
}

bool Actuators::animateValve(ServosList servo, float position, uint32_t time)
{
    Lock<FastMutex> lock(infosMutex);
    ManualValveInfo* info = getManualValve(servo);
    if (info == nullptr)
        return false;

    getModule<CanHandler>()->sendServoOpenCommand(servo, time);
    info->animateValve(position, time);
    signalTask();
    return true;
}

void Actuators::closeAllValves()
{
    Lock<FastMutex> lock(infosMutex);
    for (auto& valve : valveInfos)
        valve.closeValve();

    for (auto& valve : manualValveInfos)
        valve.closeValve();

    getModule<CanHandler>()->sendServoCloseCommand(ServosList::MAIN_OX_VALVE);
    getModule<CanHandler>()->sendServoCloseCommand(ServosList::MAIN_FUEL_VALVE);
    getModule<CanHandler>()->sendServoCloseCommand(
        ServosList::OX_VENTING_VALVE);
    getModule<CanHandler>()->sendServoCloseCommand(
        ServosList::FUEL_VENTING_VALVE);

    signalTask();
}

bool Actuators::setMaxAperture(ServosList servo, float aperture)
{
    Lock<FastMutex> lock(infosMutex);
    ValveInfo* info = getValve(servo);
    if (info == nullptr)
        return false;

    if (aperture >= 0.0 && aperture <= 1.0)
    {
        getModule<Registry>()->setUnsafe(info->valve->getMaxApertureRegKey(),
                                         aperture);
    }
    else
    {
        // What? Who would ever set this to above 100%?
        return false;
    }
    return true;
}

bool Actuators::setOpeningTime(ServosList servo, uint32_t time)
{
    Lock<FastMutex> lock(infosMutex);
    ValveInfo* info = getValve(servo);
    if (info == nullptr)
        return false;

    getModule<Registry>()->setUnsafe(info->valve->getOpeningTimeRegKey(), time);
    return true;
}

bool Actuators::isValveOpen(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ValveInfo* info = getValve(servo);
    if (info == nullptr)
        return false;

    return info->isValveOpen();
}

uint32_t Actuators::getServoOpeningTime(ServosList servo)
{
    ValveInfo* info = getValve(servo);
    if (info == nullptr)
        return 0;

    return getModule<Registry>()->getOrSetDefaultUnsafe(
        info->valve->getOpeningTimeRegKey(),
        info->valve->getDefaultOpeningTime());
}

float Actuators::getServoMaxAperture(ServosList servo)
{
    // Lock<FastMutex> lock(infosMutex);
    ValveInfo* info = getValve(servo);
    if (info == nullptr)
        return 0;

    return getModule<Registry>()->getOrSetDefaultUnsafe(
        info->valve->getMaxApertureRegKey(),
        info->valve->getDefaultMaxAperture());
}

float Actuators::getValvePosition(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ValveInfo* info = getValve(servo);
    if (info == nullptr)
        return false;

    return info->valve->getPosition();
}

Actuators::ValveState Actuators::getValveState(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);

    ValveInfo* info = getValve(servo);
    if (info == nullptr)
        return {};

    bool isOpen      = info->isValveOpen();
    auto timeToClose = 0ms;

    if (isOpen)
    {
        // Subtract 400ms to account for radio latency (empirically tested)
        auto diff = info->closeTs - Clock::now() - 400ms;
        if (diff > 0ms)
            timeToClose = duration_cast<milliseconds>(diff);
    }

    return ValveState{
        .valid       = true,
        .state       = isOpen,
        .timing      = milliseconds{getServoOpeningTime(servo)},
        .timeToClose = timeToClose,
        .aperture    = getServoMaxAperture(servo),
        .position    = info->valve->currentPosition,
    };
}

void Actuators::logValveMovement(int idx, float position)
{
    ActuatorsData data(TimestampTimer::getTimestamp(), idx, position);
    sdLogger.log(data);
}

void Actuators::startSparkPlugWithTime(uint32_t time)
{
    unsafeStartSparkPlug();
    sparkPlugCloseTs = Clock::now() + nanoseconds{msToNs(time)};
    signalTask();
}

void Actuators::stopSparkPlug()
{
    unsafeStopSparkPlug();
    sparkPlugCloseTs = noActionNeeded;
    signalTask();
}

void Actuators::toggleSparkPlug()
{
    if (sparkPlugCloseTs != noActionNeeded)
    {
        stopSparkPlug();
        return;
    }
    startSparkPlugWithTime(5000);
}

bool Actuators::isSparkSparking() { return sparkPlugCloseTs != noActionNeeded; }
void Actuators::unsafeStartSparkPlug() { spark->start(); };
void Actuators::unsafeStopSparkPlug() { spark->stop(); };

Actuators::ValveInfo* Actuators::getValve(ServosList servo)
{
    switch (servo)
    {
        case OX_VENTING_VALVE:
            return &valveInfos[0];
        case FUEL_VENTING_VALVE:
            return &valveInfos[1];
        case IGNITION_OX_VALVE:
            return &valveInfos[2];
        case IGNITION_FUEL_VALVE:
            return &valveInfos[3];

        default:
            // Oh FUCK
            LOG_ERR(logger, "Invalid servo requested");
            return nullptr;
    }
}

Actuators::ManualValveInfo* Actuators::getManualValve(ServosList servo)
{
    switch (servo)
    {
        case MAIN_OX_VALVE:
            return &manualValveInfos[0];
        case MAIN_FUEL_VALVE:
            return &manualValveInfos[1];
        case PRZ_OX_VALVE:
            return &manualValveInfos[2];
        case PRZ_FUEL_VALVE:
            return &manualValveInfos[3];

        default:
            // Oh FUCK
            LOG_ERR(logger, "Invalid servo requested");
            return nullptr;
    }
}

SignaledDeadlineTask::TimePoint Actuators::nextTaskDeadline()
{
    Lock<FastMutex> lock(infosMutex);

    // Start with the maximum value
    auto nextDeadline = TimePoint::max();

    // Get the closest deadline from all normal valves
    for (auto& info : valveInfos)
    {
        if (info.backstepTs != noActionNeeded)
            nextDeadline = std::min(nextDeadline, info.backstepTs);

        if (info.closeTs != noActionNeeded)
            nextDeadline = std::min(nextDeadline, info.closeTs);
    }

    // Get the closest deadline from all manual valves
    for (auto& info : manualValveInfos)
    {
        if (info.updateTs != noActionNeeded)
            nextDeadline = std::min(nextDeadline, info.updateTs);

        if (info.backstepTs != noActionNeeded)
            nextDeadline = std::min(nextDeadline, info.backstepTs);

        if (info.closeTs != noActionNeeded)
            nextDeadline = std::min(nextDeadline, info.closeTs);
    }

    if (sparkPlugCloseTs != noActionNeeded)
        nextDeadline = std::min(nextDeadline, sparkPlugCloseTs);

    return nextDeadline;
}

void Actuators::task()
{
    auto currentTime = Clock::now();

    Lock<FastMutex> lock(infosMutex);
    for (size_t idx = 0; idx < valveInfos.size(); idx++)
    {
        auto& info = valveInfos[idx];
        if (info.backstepTs != noActionNeeded && currentTime > info.backstepTs)
        {
            // Backstep the servo a little to avoid strain
            info.backstep();
        }
        else if (info.closeTs != noActionNeeded && currentTime <= info.closeTs)
        {
            // Open the servo only if it's not already open
            info.openValve();
            logValveMovement(idx, info.valve->currentPosition);
        }
        else if (info.closeTs != noActionNeeded && currentTime > info.closeTs)
        {
            // Close the servo only if it's not already closed
            info.closeValve();
            logValveMovement(idx, info.valve->currentPosition);
        }
    }

    for (size_t idx = 0; idx < manualValveInfos.size(); idx++)
    {
        auto& info = manualValveInfos[idx];
        if (info.backstepTs != noActionNeeded && currentTime > info.backstepTs)
        {
            // Backstep the servo a little to avoid strain
            info.backstep();
        }
        else if (info.updateTs != noActionNeeded && currentTime > info.updateTs)
        {
            // Animate servo step
            info.advanceAnimation();
            logValveMovement(valveInfos.size() + idx,
                             info.valve->currentPosition);
        }
        else if (info.closeTs != noActionNeeded && currentTime <= info.closeTs)
        {
            // Open the servo only if it's not already open
            info.openValve();
            logValveMovement(valveInfos.size() + idx,
                             info.valve->currentPosition);
        }
        else if (info.closeTs != noActionNeeded && currentTime > info.closeTs)
        {
            // Close the servo only if it's not already closed
            info.closeValve();
            logValveMovement(valveInfos.size() + idx,
                             info.valve->currentPosition);
        }
    }

    // handle spark plug timing
    if (sparkPlugCloseTs != noActionNeeded && currentTime > sparkPlugCloseTs)
        stopSparkPlug();

    // Check if we reached the inactivity timeout and should vent
    if (currentTime >= safetyVentingTs)
    {
        openValveWithTime(
            ServosList::OX_VENTING_VALVE,
            milliseconds{Config::Servos::SAFETY_VENTING_DURATION}.count());

        openValveWithTime(
            ServosList::FUEL_VENTING_VALVE,
            milliseconds{Config::Servos::SAFETY_VENTING_DURATION}.count());

        // Reset the safety venting timestamp
        safetyVentingTs = currentTime + Config::Servos::SAFETY_VENTING_TIMEOUT;
    }
}

