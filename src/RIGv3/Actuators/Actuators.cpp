/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Niccolò Betto, Pietro Bortolus, Riccardo Sironi
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

#include <RIGv3/Actuators/ActuatorsData.h>
#include <RIGv3/Configs/ActuatorsConfig.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>

#include "ActuatorsMacros.h"

using namespace std::chrono;
using namespace Boardcore;
using namespace miosix;
using namespace Common;
using namespace RIGv3;

const Actuators::TimePoint Actuators::ValveClosed = TimePoint{};

bool Actuators::ValveInfo::isServoOpen()
{
    if (valve->getType() == ValveType::SERVO)
        return closeTs != ValveClosed;
    else
        // if the valve is closed getPosition() is going to return 0. Otherwise,
        // the function returns true.
        // We also check for floating point inaccuracy
        return valve->getPosition();
}

void Actuators::ValveInfo::openServoWithTime(float position, uint32_t time)
{
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

    float delta        = std::abs(position - valve->currentPosition);
    auto backstepDelay = milliseconds{
        static_cast<int>(Config::Servos::SERVO_BACKSTEP_DELAY.count() * delta)};

    valve->currentPosition = position;

    auto currentTime = Clock::now();

    closeTs    = currentTime + nanoseconds{msToNs(time)};
    backstepTs = currentTime + backstepDelay;

    // Reset animation in case it was interrupted
    updateTs       = ValveClosed;
    animationEndTs = ValveClosed;

    valve->setPosition(valve->currentPosition);
}

void Actuators::ValveInfo::openServoWithTime(uint32_t time)
{
    valve->direction = Valve::Direction::OPEN;
    auto currentTime = Clock::now();

    closeTs = currentTime + nanoseconds{msToNs(time)};
    backstepTs =
        currentTime + milliseconds{Config::Servos::SERVO_BACKSTEP_DELAY};

    // Set opening position
    valve->currentPosition = std::min(1.0f, std::max(0.0f, getMaxAperture()));

    // Reset animation in case it was interrupted
    updateTs       = ValveClosed;
    animationEndTs = ValveClosed;

    valve->setPosition(valve->currentPosition);

    const uint8_t openingEvent = valve->getOpeningEvent();

    if (openingEvent != 0)
        EventBroker::getInstance().post(openingEvent, TOPIC_MOTOR);
}

void Actuators::ValveInfo::animateServo(float position, uint32_t time)
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

    auto now       = Clock::now();
    animationEndTs = now + nanoseconds{msToNs(time)};
    updateTs       = now + Config::Servos::ANIMATION_UPDATE_PERIOD;
    backstepTs     = now + nanoseconds{msToNs(time)} + backstepDelay;
    closeTs = ValveClosed;  // The animate function does not close the servo
                            // automatically

    float stepCount = time / Config::Servos::ANIMATION_UPDATE_PERIOD.count();
    animationStep   = delta / stepCount;
}

void Actuators::ValveInfo::closeServo()
{
    float delta        = valve->currentPosition;
    auto backstepDelay = milliseconds{
        static_cast<int>(Config::Servos::SERVO_BACKSTEP_DELAY.count() * delta)};

    closeTs    = ValveClosed;
    backstepTs = Clock::now() + backstepDelay;

    valve->currentPosition = 0.0f;
    valve->direction       = Valve::Direction::CLOSE;

    // Reset animation in case it was interrupted
    updateTs       = ValveClosed;
    animationEndTs = ValveClosed;

    valve->setPosition(valve->currentPosition);

    const uint8_t closingEvent = valve->getClosingEvent();

    if (closingEvent != 0)
        EventBroker::getInstance().post(closingEvent, TOPIC_MOTOR);
}

void Actuators::ValveInfo::backstep()
{
    valve->backstep();
    backstepTs = ValveClosed;  // Reset backstep time
}

void Actuators::ValveInfo::advanceAnimation()
{
    // If an animation is in progress, advance it
    if (animationEndTs != ValveClosed)
    {
        auto currentTime = Clock::now();
        if (currentTime <= animationEndTs)
        {
            // Update the servo position
            valve->currentPosition += animationStep;

            // Clamp position
            valve->currentPosition =
                std::min(1.0f, std::max(0.0f, valve->currentPosition));

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

    valve->setPosition(valve->currentPosition);
}

float Actuators::ValveInfo::getMaxAperture()
{
    getModule<Registry>()->getOrSetDefaultUnsafe(
        valve->getMaxApertureRegKey(), valve->getDefaultMaxAperture());
}

uint32_t Actuators::ValveInfo::getOpeningTime()
{
    return getModule<Registry>()->getOrSetDefaultUnsafe(
        valve->getOpeningTimeRegKey(), valve->getDefaultOpeningTime());
}

bool Actuators::ValveInfo::setMaxAperture(float aperture)
{
    if (aperture >= 0.0 && aperture <= 1.0)
    {
        getModule<Registry>()->setUnsafe(valve->getMaxApertureRegKey(),
                                         aperture);
        return true;
    }
    else
    {
        // What? Who would ever set this to above 100%?
        return false;
    }
}

bool Actuators::ValveInfo::setOpeningTime(uint32_t time)
{
    getModule<Registry>()->setUnsafe(valve->getOpeningTimeRegKey(), time);
    return true;
}

Actuators::Actuators(I2C& i2c, I2CDriver::I2CSlaveConfig i2cConfig0,
                     I2CDriver::I2CSlaveConfig i2cConfig1)
    : SignaledDeadlineTask(miosix::STACK_DEFAULT_FOR_PTHREAD,
                           BoardScheduler::actuatorsPriority()),
      expander0(i2c, i2cConfig0), expander1(i2c, i2cConfig1),

      prz_3wayValveInfo(MAKE_SIMPLE_PCA_SERVO_VALVE(
          PRZ_3W, expander0, PCA9685Utils::Channel::CHANNEL_0)),

      spark(std::make_unique<SparkPlug>(
          MIOSIX_IGNITER_TIM, (uint16_t)50,
          TimerUtils::Channel::MIOSIX_IGNITER_CHANNEL))

{
    infos.push_back(MAKE_PCA_SERVO_VALVE(PRZ_FIL, expander0,
                                         PCA9685Utils::Channel::CHANNEL_1));
    infos.push_back(MAKE_PCA_SERVO_VALVE(PRZ_REL, expander0,
                                         PCA9685Utils::Channel::CHANNEL_2));
    infos.push_back(MAKE_PCA_SERVO_VALVE(OX_FIL, expander0,
                                         PCA9685Utils::Channel::CHANNEL_3));
    infos.push_back(MAKE_PCA_SERVO_VALVE(OX_REL, expander0,
                                         PCA9685Utils::Channel::CHANNEL_4));

    // infos.push_back(MAKE_SOLENOID_VALVE(PRZ_DET, ));
    // infos.push_back(MAKE_SOLENOID_VALVE(OX_DET, ));

    infos.push_back(MAKE_PCA_SERVO_VALVE(PRZ_OX, expander1,
                                         PCA9685Utils::Channel::CHANNEL_0));
    infos.push_back(MAKE_PCA_SERVO_VALVE(PRZ_FUEL, expander1,
                                         PCA9685Utils::Channel::CHANNEL_1));

    infos.push_back(MAKE_PCA_SERVO_VALVE(OX_VEN, expander1,
                                         PCA9685Utils::Channel::CHANNEL_2));
    infos.push_back(MAKE_PCA_SERVO_VALVE(FUEL_VEN, expander1,
                                         PCA9685Utils::Channel::CHANNEL_3));

    infos.push_back(MAKE_PCA_SERVO_VALVE(MAIN_OX, expander1,
                                         PCA9685Utils::Channel::CHANNEL_4));
    infos.push_back(MAKE_PCA_SERVO_VALVE(MAIN_FUEL, expander1,
                                         PCA9685Utils::Channel::CHANNEL_5));
    infos.push_back(
        MAKE_SOLENOID_VALVE(IGN_OX, actuators::oxSolenoid::getPin()));
    infos.push_back(
        MAKE_SOLENOID_VALVE(IGN_FUEL, actuators::fuelSolenoid::getPin()));

    closeAllServos();

    prz_3wayValveInfo.closeServo();
    spark->stop();
}

bool Actuators::isStarted() { return started; }

bool Actuators::start()
{
    // Enable all servos and close them to force a backstep
    for (auto& info : infos)
    {
        info.valve->enable();
        info.closeServo();
        info.backstepTs = Clock::now() + milliseconds{2000};
    }

    prz_3wayValveInfo.valve->enable();
    prz_3wayValveInfo.closeServo();

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
    ValveInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    if (info->closeTs == ValveClosed)
    {
        uint32_t time = info->getOpeningTime();

        // The servo is closed, open it
        getModule<CanHandler>()->sendServoOpenCommand(servo, time);
        openServoWithTime(servo, time);
    }
    else
    {
        // The servo is open, close it
        getModule<CanHandler>()->sendServoCloseCommand(servo);
        closeServo(servo);
    }

    signalTask();
    return true;
}

bool Actuators::openServo(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ValveInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    uint32_t time = info->getOpeningTime();

    getModule<CanHandler>()->sendServoOpenCommand(servo, time);
    openServoWithTime(servo, time);
    signalTask();
    return true;
}

bool Actuators::openServoWithTime(ServosList servo, uint32_t time)
{
    Lock<FastMutex> lock(infosMutex);
    ValveInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    getModule<CanHandler>()->sendServoOpenCommand(servo, time);
    moveServoWithTime(servo, info->getMaxAperture(), time);
    signalTask();
    return true;
}

bool Actuators::moveServoWithTime(ServosList servo, float position,
                                  uint32_t time)
{
    Lock<FastMutex> lock(infosMutex);
    ValveInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    getModule<CanHandler>()->sendServoOpenCommand(servo, time);
    info->openServoWithTime(position, time);
    signalTask();
    return true;
}

bool Actuators::animateServo(ServosList servo, float position, uint32_t time)
{
    Lock<FastMutex> lock(infosMutex);
    ValveInfo* info = getServo(servo);
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
    ValveInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    getModule<CanHandler>()->sendServoCloseCommand(servo);
    info->closeServo();
    signalTask();
    return true;
}

// TODO: rename to close Valves and check logic for solenoid valves
void Actuators::closeAllServos()
{
    Lock<FastMutex> lock(infosMutex);
    for (uint8_t idx = 0; idx < 10; idx++)
        infos[idx].closeServo();

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
    ValveInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    if (aperture >= 0.0 && aperture <= 1.0)
    {
        info->setMaxAperture(aperture);
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
    ValveInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    info->setOpeningTime(time);
}

bool Actuators::isServoOpen(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ValveInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    return info->isServoOpen();
}

uint32_t Actuators::getServoOpeningTime(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ValveInfo* info = getServo(servo);
    if (info == nullptr)
        return 0;

    return info->getOpeningTime();
}

float Actuators::getServoMaxAperture(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ValveInfo* info = getServo(servo);
    if (info == nullptr)
        return 0;

    return info->getMaxAperture();
}

Actuators::ValveState Actuators::getValveInfo(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);

    ValveInfo* info = getServo(servo);
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

        currentPosition = info->valve->currentPosition;
    }

    return ValveState{
        .valid       = true,
        .state       = isOpen,
        .timing      = milliseconds{info->getOpeningTime()},
        .timeToClose = timeToClose,
        .aperture    = info->getMaxAperture(),
        .position    = info->valve->currentPosition,
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

        prz_3wayValveInfo.valve->direction       = Valve::Direction::OPEN;
        prz_3wayValveInfo.valve->currentPosition = 1.0f;

        prz_3wayValveInfo.backstepTs =
            currentTime + nanoseconds{Config::Servos::SERVO_BACKSTEP_DELAY};

        prz_3wayValveInfo.valve->setPosition(1.0f);
    }
    else
        prz_3wayValveInfo.closeServo();

    signalTask();
}

bool Actuators::get3wayValveState() { return prz_3wayValveState; }

uint32_t Actuators::getServoOpeningTime(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ValveInfo* info = getServo(servo);
    if (info == nullptr)
        return 0;

    return info->getOpeningTime();
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
    sparkPlugCloseTs = ValveClosed;
    signalTask();
}

void Actuators::toggleSparkPlug()
{
    if (sparkPlugCloseTs != ValveClosed)
    {
        stopSparkPlug();
        return;
    }
    startSparkPlugWithTime(5000);
}

bool Actuators::isSparkSparking() { return sparkPlugCloseTs != ValveClosed; }

void Actuators::inject(DependencyInjector& injector)
{
    Super::inject(injector);
    for (ValveInfo& info : infos)
        info.inject(injector);
    prz_3wayValveInfo.inject(injector);
}

Actuators::ValveInfo* Actuators::getServo(ServosList servo)
{
    switch (servo)
    {
        case PRZ_FILLING_VALVE:  // OX_FIL
            return &infos[0];
        case PRZ_RELEASE_VALVE:  // OX_REL
            return &infos[1];
        case OX_FILLING_VALVE:  // OX_FIL
            return &infos[2];
        case OX_RELEASE_VALVE:  // OX_REL
            return &infos[3];

            /* case OX_DETACH_SERVO:  // OX_DET
                return &infos[2];
            case PRZ_DETACH_SERVO:  // PRZ_DET
                return &infos[3]; */

        case PRZ_OX_VALVE:  // N2_FIL
            return &infos[4];
        case PRZ_FUEL_VALVE:  // N2_REL
            return &infos[5];

        case OX_VENTING_VALVE:  // N2_DET
            return &infos[6];
        case FUEL_VENTING_VALVE:  // N2_DET
            return &infos[7];

        case MAIN_OX_VALVE:  // MAIN_OX
            return &infos[8];
        case MAIN_FUEL_VALVE:  // MAIN_FUEL
            return &infos[9];
        case IGNITION_OX_VALVE:  // IGN_OX
            return &infos[10];
        case IGNITION_FUEL_VALVE:  // IGN_FUEL
            return &infos[11];

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
            info.advanceAnimation();
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

    // handle spark plug timing
    if (currentTime > sparkPlugCloseTs && sparkPlugCloseTs != ValveClosed)
        stopSparkPlug();
}

void Actuators::unsafeStartSparkPlug() { spark->start(); };
void Actuators::unsafeStopSparkPlug() { spark->stop(); };
