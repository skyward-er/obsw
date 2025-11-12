/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto, Pietro Bortolus
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
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>

#include "ActuatorsMacros.h"

using namespace std::chrono;
using namespace Boardcore;
using namespace miosix;
using namespace Common;
using namespace RIGv2;

const Actuators::TimePoint Actuators::ValveClosed = TimePoint{};

void Actuators::ServoInfo::openServoWithTime(float position, uint32_t time)
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
    auto backstepDelay = milliseconds{
        static_cast<int>(Config::Servos::SERVO_BACKSTEP_DELAY.count() * delta)};

    currentPosition = position;

    auto currentTime = Clock::now();

    closeTs    = currentTime + nanoseconds{msToNs(time)};
    backstepTs = currentTime + backstepDelay;

    // Reset animation in case it was interrupted
    updateTs       = ValveClosed;
    animationEndTs = ValveClosed;

    servo->setPosition(scalePosition(currentPosition));

    if (config.openingEvent != 0)
        EventBroker::getInstance().post(config.openingEvent, TOPIC_MOTOR);
}

void Actuators::ServoInfo::openServoWithTime(uint32_t time)
{
    auto currentTime = Clock::now();

    closeTs = currentTime + nanoseconds{msToNs(time)};
    backstepTs =
        currentTime + nanoseconds{Config::Servos::SERVO_BACKSTEP_DELAY};

    // Set opening position
    currentPosition = getMaxAperture();

    // Reset animation in case it was interrupted
    updateTs       = ValveClosed;
    animationEndTs = ValveClosed;

    servo->setPosition(scalePosition(currentPosition));

    if (config.openingEvent != 0)
        EventBroker::getInstance().post(config.openingEvent, TOPIC_MOTOR);
}

void Actuators::ServoInfo::animateServo(float position, uint32_t time)
{
    // Nothing to animate
    if (currentPosition == position)
        return;

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

    float delta        = position - currentPosition;
    auto backstepDelay = milliseconds{static_cast<int>(
        Config::Servos::SERVO_BACKSTEP_DELAY.count() * std::abs(delta))};

    auto now       = Clock::now();
    animationEndTs = now + nanoseconds{msToNs(time)};
    updateTs       = now + Config::Servos::ANIMATION_UPDATE_PERIOD;
    backstepTs     = now + nanoseconds{msToNs(time)} + backstepDelay;
    closeTs = ValveClosed;  // The animate function does not close the servo
                            // automatically

    float stepCount = time / Config::Servos::ANIMATION_UPDATE_PERIOD.count();
    animationStep   = delta / stepCount;

    // Debugging
    /* printf(
        "set animation with max aperture: %.5f, current position: %5.f, delta: "
        "%.5f, step count: %.5f"
        "animation step: %.5f\n",
        position, currentPosition, delta, stepCount, animationStep); */
}

void Actuators::ServoInfo::closeServo()
{
    float delta        = currentPosition;
    auto backstepDelay = milliseconds{
        static_cast<int>(Config::Servos::SERVO_BACKSTEP_DELAY.count() * delta)};

    closeTs    = ValveClosed;
    backstepTs = Clock::now() + backstepDelay;

    currentPosition = 0.0f;
    direction       = Direction::CLOSE;

    // Reset animation in case it was interrupted
    updateTs       = ValveClosed;
    animationEndTs = ValveClosed;

    servo->setPosition(scalePosition(currentPosition));

    if (config.closingEvent != 0)
        EventBroker::getInstance().post(config.closingEvent, TOPIC_MOTOR);
}

void Actuators::ServoInfo::backstep()
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

    servo->setPosition(scalePosition(currentPosition));
    backstepTs = ValveClosed;  // Reset backstep time
}

void Actuators::ServoInfo::move()
{
    // If an animation is in progress, advance it
    if (animationEndTs != ValveClosed)
    {
        auto currentTime = Clock::now();
        if (currentTime <= animationEndTs)
        {
            // Update the servo position
            currentPosition += animationStep;
            // Schedule the next update
            updateTs = currentTime + Config::Servos::ANIMATION_UPDATE_PERIOD;
        }
        else
        {
            // If the animation is done, reset animation times
            animationEndTs = ValveClosed;
            updateTs       = ValveClosed;
        }
    }

    // printf("\tMoving valve to position %.3f\n", currentPosition);

    servo->setPosition(scalePosition(currentPosition));
}

float Actuators::ServoInfo::scalePosition(float position)
{
    position *= config.limit;
    if (config.flipped)
        position = 1.0f - position;

    return position;
}

void Actuators::ServoInfo::unsafeSetServoPosition(float position)
{
    // Check that the servo is actually there, just to be safe
    if (!servo)
        return;

    position *= config.limit;
    if (config.flipped)
        position = 1.0f - position;

    servo->setPosition(position);
}

float Actuators::ServoInfo::getServoPosition()
{
    // Check that the servo is actually there, just to be safe
    if (!servo)
        return 0.0f;

    float position = servo->getPosition();
    if (config.flipped)
        position = 1.0f - position;

    position /= config.limit;
    return position;
}

float Actuators::ServoInfo::getMaxAperture()
{
    return getModule<Registry>()->getOrSetDefaultUnsafe(
        config.maxApertureRegKey, config.defaultMaxAperture);
}

uint32_t Actuators::ServoInfo::getOpeningTime()
{
    return getModule<Registry>()->getOrSetDefaultUnsafe(
        config.openingTimeRegKey, config.defaultOpeningTime);
}

bool Actuators::ServoInfo::isServoOpen() { return closeTs != ValveClosed; }

bool Actuators::ServoInfo::setMaxAperture(float aperture)
{
    if (aperture >= 0.0 && aperture <= 1.0)
    {
        getModule<Registry>()->setUnsafe(config.maxApertureRegKey, aperture);
        return true;
    }
    else
    {
        // What? Who would ever set this to above 100%?
        return false;
    }
}

bool Actuators::ServoInfo::setOpeningTime(uint32_t time)
{
    getModule<Registry>()->setUnsafe(config.openingTimeRegKey, time);
    return true;
}

Actuators::Actuators()
    : SignaledDeadlineTask(miosix::STACK_DEFAULT_FOR_PTHREAD,
                           BoardScheduler::actuatorsPriority()),
      infos{MAKE_SERVO(OX_FIL),        MAKE_SERVO(OX_REL),
            MAKE_SMALL_SERVO(PRZ_FIL), MAKE_SMALL_SERVO(PRZ_REL),
            MAKE_SERVO(PRZ_FUEL),      MAKE_SERVO(PRZ_OX),
            MAKE_SERVO(OX_VEN),        MAKE_SERVO(FUEL_VEN),
            MAKE_SERVO(MAIN_OX),       MAKE_SERVO(MAIN_FUEL)},
      prz_3wayValveInfo(MAKE_SIMPLE_SERVO(PRZ_3W))
{
    for (auto& servo : infos)
        servo.unsafeSetServoPosition(0.0f);

    prz_3wayValveInfo.unsafeSetServoPosition(0.0f);
    unsafeCloseChamber();
}

bool Actuators::isStarted() { return started; }

bool Actuators::start()
{
    // Enable all servos and close them to force a backstep
    for (auto& info : infos)
    {
        info.servo->enable();
        info.closeServo();
    }

    prz_3wayValveInfo.servo->enable();
    prz_3wayValveInfo.closeServo();

    if (!SignaledDeadlineTask::start())
    {
        LOG_ERR(logger, "Failed to start Actuators task");
        return false;
    }

    started = true;
    return true;
}

void Actuators::armLightOn() { relays::armLight::low(); }
void Actuators::armLightOff() { relays::armLight::high(); }

void Actuators::igniterOn() { relays::ignition::low(); }
void Actuators::igniterOff() { relays::ignition::high(); }

void Actuators::clacsonOn() { relays::clacson::low(); }
void Actuators::clacsonOff() { relays::clacson::high(); }

bool Actuators::wiggleServo(ServosList servo)
{
    // Special handling for the 3-way valve
    if (servo == PRZ_3WAY_VALVE)
    {
        // Toggle the valve to the current opposite state
        auto state = get3wayValveState();

        set3wayValveState(!state);
        Thread::sleep(1000);
        set3wayValveState(state);

        return true;
    }

    // Wiggle means open the servo for 1s
    return openServoWithTime(servo, 1000);
}

bool Actuators::toggleServo(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    if (info->closeTs == ValveClosed)
    {
        uint32_t time = info->getOpeningTime();

        // The servo is closed, open it
        getModule<CanHandler>()->sendServoOpenCommand(servo, time);
        info->openServoWithTime(time);
    }
    else
    {
        // The servo is open, close it
        getModule<CanHandler>()->sendServoCloseCommand(servo);
        info->closeServo();
    }

    signalTask();
    return true;
}

bool Actuators::openServo(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    uint32_t time = info->getOpeningTime();
    getModule<CanHandler>()->sendServoOpenCommand(servo, time);
    info->openServoWithTime(time);
    signalTask();
    return true;
}

bool Actuators::moveServo(ServosList servo, float position)
{
    uint32_t time = Config::Servos::MOVE_SERVO_TIMEOUT.count();

    Lock<FastMutex> lock(infosMutex);
    ServoInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    getModule<CanHandler>()->sendServoOpenCommand(servo, time);
    info->openServoWithTime(position, time);
    signalTask();
    return true;
}

bool Actuators::moveServoWithTime(ServosList servo, float position,
                                  uint32_t time)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    getModule<CanHandler>()->sendServoOpenCommand(servo, time);
    info->openServoWithTime(position, time);
    signalTask();
    return true;
}

bool Actuators::openServoWithTime(ServosList servo, uint32_t time)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    getModule<CanHandler>()->sendServoOpenCommand(servo, time);
    info->openServoWithTime(time);
    signalTask();
    return true;
}

bool Actuators::animateServo(ServosList servo, float position, uint32_t time)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    getModule<CanHandler>()->sendServoOpenCommand(servo, time);
    info->animateServo(position, time);
    signalTask();
    return true;
}

bool Actuators::closeServo(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    getModule<CanHandler>()->sendServoCloseCommand(servo);
    info->closeServo();
    signalTask();
    return true;
}

void Actuators::closeAllServos()
{
    Lock<FastMutex> lock(infosMutex);
    for (uint8_t idx = 0; idx < 10; idx++)
        infos[idx].closeServo();

    // TODO: figure out which valves need a can message to be closed
    getModule<CanHandler>()->sendServoCloseCommand(ServosList::MAIN_FUEL_VALVE);
    getModule<CanHandler>()->sendServoCloseCommand(ServosList::MAIN_OX_VALVE);
    getModule<CanHandler>()->sendServoCloseCommand(
        ServosList::OX_VENTING_VALVE);

    signalTask();
}

bool Actuators::setMaxAperture(ServosList servo, float aperture)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    return info->setMaxAperture(aperture);
}

bool Actuators::setOpeningTime(ServosList servo, uint32_t time)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    return info->setOpeningTime(time);
}

bool Actuators::isServoOpen(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    return info->isServoOpen();
}

Actuators::ValveInfo Actuators::getValveInfo(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);

    ServoInfo* info = getServo(servo);
    if (info == nullptr)
        return {};

    bool isOpen           = info->isServoOpen();
    auto timeToClose      = 0ms;
    float currentPosition = 0.0f;

    if (isOpen)
    {
        // Subtract 400ms to account for radio latency (empirically tested)
        auto diff = info->closeTs - Clock::now() - 400ms;
        if (diff > 0ms)
            timeToClose = duration_cast<milliseconds>(diff);

        currentPosition = info->currentPosition;
    }

    return ValveInfo{
        .valid       = true,
        .state       = isOpen,
        .timing      = milliseconds{info->getOpeningTime()},
        .timeToClose = timeToClose,
        .aperture    = info->getMaxAperture(),
        .position    = currentPosition,
    };
}

void Actuators::set3wayValveState(bool state)
{
    prz_3wayValveState = state;
    if (state)
    {
        // TODO: This part of the code sucks ass, openServoWithTime cannot be
        // called for the 3way valve since it is a simple servo so this
        // workaround is used instead

        auto currentTime = Clock::now();

        prz_3wayValveInfo.direction       = ServoInfo::Direction::OPEN;
        prz_3wayValveInfo.currentPosition = 1.0f;

        prz_3wayValveInfo.backstepTs =
            currentTime + nanoseconds{Config::Servos::SERVO_BACKSTEP_DELAY};

        prz_3wayValveInfo.unsafeSetServoPosition(1.0f);
    }
    else
        prz_3wayValveInfo.closeServo();

    signalTask();
}

bool Actuators::get3wayValveState() { return prz_3wayValveState; }

void Actuators::openChamberWithTime(uint32_t time)
{
    Lock<FastMutex> lock(infosMutex);
    chamberCloseTs = Clock::now() + nanoseconds{msToNs(time)};
    signalTask();
}

void Actuators::closeChamber()
{
    Lock<FastMutex> lock(infosMutex);
    chamberCloseTs = ValveClosed;
    signalTask();
}

bool Actuators::isChamberOpen()
{
    Lock<FastMutex> lock(infosMutex);
    return chamberCloseTs != ValveClosed;
}

uint32_t Actuators::getServoOpeningTime(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo* info = getServo(servo);
    if (info == nullptr)
        return 0;

    return info->getOpeningTime();
}

float Actuators::getServoMaxAperture(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo* info = getServo(servo);
    if (info == nullptr)
        return 0;

    return info->getMaxAperture();
}

void Actuators::inject(DependencyInjector& injector)
{
    Super::inject(injector);
    for (ServoInfo& info : infos)
        info.inject(injector);
}

Actuators::ServoInfo* Actuators::getServo(ServosList servo)
{
    switch (servo)
    {
        case OX_FILLING_VALVE:  // OX_FIL
            return &infos[0];
        case OX_RELEASE_VALVE:  // OX_REL
            return &infos[1];
        /* case OX_DETACH_SERVO:  // OX_DET
            return &infos[2]; */
        case PRZ_FILLING_VALVE:  // PRZ_FIL
            return &infos[2];
        case PRZ_RELEASE_VALVE:  // PRZ_REL
            /* return &infos[4];
        case PRZ_DETACH_SERVO:  // PRZ_DET */
            return &infos[3];
        case PRZ_FUEL_VALVE:  // PRZ_FUEL
            return &infos[4];
        case PRZ_OX_VALVE:  // PRZ_OX
            return &infos[5];
        case OX_VENTING_VALVE:  // OX_VEN
            return &infos[6];
        case FUEL_VENTING_VALVE:  // FUEL_VEN
            return &infos[7];
        case MAIN_OX_VALVE:  // MAIN_OX
            return &infos[8];
        case MAIN_FUEL_VALVE:  // MAIN_FUEL
            return &infos[9];

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

void Actuators::unsafeOpenChamber() { relays::nitrogen::low(); }

void Actuators::unsafeCloseChamber() { relays::nitrogen::high(); }

SignaledDeadlineTask::TimePoint Actuators::nextTaskDeadline()
{
    Lock<FastMutex> lock(infosMutex);

    // Start with the maximum value
    auto nextDeadline = TimePoint::max();

    // Get the closest deadline from all valves
    for (auto& info : infos)
    {
        if (info.updateTs != ValveClosed)
            nextDeadline = std::min(nextDeadline, info.updateTs);

        if (info.backstepTs != ValveClosed)
            nextDeadline = std::min(nextDeadline, info.backstepTs);

        if (info.closeTs != ValveClosed)
            nextDeadline = std::min(nextDeadline, info.closeTs);
    }

    // 3-way valve is not a timed valve, only needs backstep handling
    if (prz_3wayValveInfo.backstepTs != ValveClosed)
        nextDeadline = std::min(nextDeadline, prz_3wayValveInfo.backstepTs);

    if (chamberCloseTs != ValveClosed)
        nextDeadline = std::min(nextDeadline, chamberCloseTs);

    return nextDeadline;
}

void Actuators::task()
{
    auto currentTime = Clock::now();

    Lock<FastMutex> lock(infosMutex);
    for (auto& info : infos)
    {
        if (currentTime > info.backstepTs && info.backstepTs != ValveClosed)
        {
            // Backstep the servo a little to avoid strain
            info.backstep();
        }
        else if (currentTime > info.updateTs && info.updateTs != ValveClosed)
        {
            // Animate servo step
            info.move();
        }
        else if (currentTime > info.closeTs && info.closeTs != ValveClosed)
        {
            // Close the servo
            info.closeServo();
        }
    }

    if (currentTime > prz_3wayValveInfo.backstepTs &&
        prz_3wayValveInfo.backstepTs != ValveClosed)
    {
        // Backstep the 3-way valve servo a little to avoid strain
        prz_3wayValveInfo.backstep();
    }
}
