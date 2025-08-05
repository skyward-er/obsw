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

void Actuators::ServoInfo::openServoWithTime(uint32_t time)
{
    auto currentTime = Clock::now();

    closeTs = currentTime + nanoseconds{msToNs(time)};
    backstepTs =
        currentTime + nanoseconds{Config::Servos::SERVO_BACKSTEP_DELAY};

    if (config.openingEvent != 0)
        EventBroker::getInstance().post(config.openingEvent, TOPIC_MOTOR);
}

void Actuators::ServoInfo::closeServo()
{
    closeTs = ValveClosed;
    backstepTs =
        Clock::now() + nanoseconds{Config::Servos::SERVO_BACKSTEP_DELAY};

    if (config.closingEvent != 0)
        EventBroker::getInstance().post(config.closingEvent, TOPIC_MOTOR);
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
      infos{MAKE_SERVO(OX_FIL),       MAKE_SERVO(OX_REL),
            MAKE_SMALL_SERVO(OX_DET), MAKE_SMALL_SERVO(N2_FIL),
            MAKE_SMALL_SERVO(N2_REL), MAKE_SMALL_SERVO(N2_DET),
            MAKE_SERVO(NITR),         MAKE_SERVO(OX_VEN),
            MAKE_SMALL_SERVO(N2_QUE), MAKE_SERVO(MAIN)},
      n2_3wayValveInfo(MAKE_SIMPLE_SERVO(N2_3W))
{
    for (auto& servo : infos)
        servo.unsafeSetServoPosition(0.0f);

    n2_3wayValveInfo.unsafeSetServoPosition(0.0f);
}

bool Actuators::isStarted() { return started; }

bool Actuators::start()
{
    // Enable all servos
    for (ServoInfo& info : infos)
        info.servo->enable();

    n2_3wayValveInfo.servo->enable();

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
    if (servo == N2_3WAY_VALVE)
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

    getModule<CanHandler>()->sendServoCloseCommand(ServosList::MAIN_VALVE);
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

bool Actuators::isCanServoOpen(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);

    if (servo == ServosList::MAIN_VALVE)
        return canMainOpen;
    else if (servo == ServosList::NITROGEN_VALVE)
        return canNitrogenOpen;
    else if (servo == ServosList::OX_VENTING_VALVE)
        return canOxVentingOpen;
    else if (servo == ServosList::N2_QUENCHING_VALVE)
        return canN2QuenchingOpen;
    else
        return false;
}

Actuators::ValveInfo Actuators::getValveInfo(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);

    ServoInfo* info = getServo(servo);
    if (info == nullptr)
        return {};

    auto timeToClose = 0ms;
    if (info->closeTs != ValveClosed)
    {
        // Subtract 400ms to account for radio latency (empirically tested)
        auto diff = info->closeTs - Clock::now() - 400ms;
        if (diff > 0ms)
            timeToClose = duration_cast<milliseconds>(diff);
    }

    return ValveInfo{
        .state       = info->isServoOpen(),
        .timing      = milliseconds{info->getOpeningTime()},
        .timeToClose = timeToClose,
        .aperture    = info->getMaxAperture(),
    };
}

void Actuators::set3wayValveState(bool state)
{
    n2_3wayValveState        = state;
    n2_3wayValveStateChanged = true;
    signalTask();
}

bool Actuators::get3wayValveState() { return n2_3wayValveState; }

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

void Actuators::setCanServoOpen(ServosList servo, bool open)
{
    Lock<FastMutex> lock(infosMutex);
    if (servo == ServosList::MAIN_VALVE)
        canMainOpen = open;
    else if (servo == ServosList::NITROGEN_VALVE)
        canNitrogenOpen = open;
    else if (servo == ServosList::OX_VENTING_VALVE)
        canOxVentingOpen = open;
    else if (servo == ServosList::N2_QUENCHING_VALVE)
        canN2QuenchingOpen = open;
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
        case OX_DETACH_SERVO:  // OX_DET
            return &infos[2];
        case N2_FILLING_VALVE:  // N2_FIL
            return &infos[3];
        case N2_RELEASE_VALVE:  // N2_REL
            return &infos[4];
        case N2_DETACH_SERVO:  // N2_DET
            return &infos[5];
        case NITROGEN_VALVE:  // NITR
            return &infos[6];
        case OX_VENTING_VALVE:  // OX_VEN
            return &infos[7];
        case N2_QUENCHING_VALVE:  // N2_QUE
            return &infos[8];
        case MAIN_VALVE:  // MAIN
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
    for (uint8_t idx = 0; idx < infos.size(); idx++)
    {
        if (infos[idx].closeTs != ValveClosed &&
            infos[idx].closeTs < nextDeadline)
            nextDeadline = infos[idx].closeTs;

        if (infos[idx].backstepTs != ValveClosed &&
            infos[idx].backstepTs < nextDeadline)
            nextDeadline = infos[idx].backstepTs;
    }

    // 3-way valve is not a timed valve, only needs backstep handling
    if (n2_3wayValveInfo.backstepTs != ValveClosed &&
        n2_3wayValveInfo.backstepTs < nextDeadline)
        nextDeadline = n2_3wayValveInfo.backstepTs;

    if (chamberCloseTs != ValveClosed && chamberCloseTs < nextDeadline)
        nextDeadline = chamberCloseTs;

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
                unsafeSetServoPosition(idx, infos[idx].getMaxAperture());
            }
            else
            {
                // Backstep the valve a little to avoid strain
                unsafeSetServoPosition(
                    idx, infos[idx].getMaxAperture() *
                             (1.0 - Config::Servos::SERVO_BACKSTEP_AMOUNT));

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
                unsafeSetServoPosition(
                    idx, infos[idx].getMaxAperture() *
                             Config::Servos::SERVO_BACKSTEP_AMOUNT);

                infos[idx].backstepTs = ValveClosed;  // Reset backstep time
            }
        }
    }

    // Handle nitrogen logic
    if (currentTime < chamberCloseTs)
    {
        unsafeOpenChamber();
    }
    else
    {
        chamberCloseTs = ValveClosed;
        unsafeCloseChamber();
    }

    // Handle the 3-way valve
    if (n2_3wayValveStateChanged)
    {
        if (n2_3wayValveState)
        {
            // Open servo method called for logging and to set backstep time
            n2_3wayValveInfo.openServoWithTime(0);
            n2_3wayValveInfo.unsafeSetServoPosition(1.0f);
        }
        else
        {
            // Close servo method called for logging and to set backstep time
            n2_3wayValveInfo.closeServo();
            n2_3wayValveInfo.unsafeSetServoPosition(0.0f);
        }

        n2_3wayValveStateChanged = false;
    }

    if (currentTime > n2_3wayValveInfo.backstepTs)
    {
        auto backstepPosition =
            n2_3wayValveState ? 1.0f - Config::Servos::SERVO_BACKSTEP_AMOUNT
                              : 0.0f + Config::Servos::SERVO_BACKSTEP_AMOUNT;
        n2_3wayValveInfo.unsafeSetServoPosition(backstepPosition);

        n2_3wayValveInfo.backstepTs = ValveClosed;  // Reset backstep time
    }

    // Wiggle the valve a little
}
