/* Copyright (c) 2025 Skyward Experimental Rocketry
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

#include <RIGv2/Actuators/ActuatorsData.h>
#include <RIGv2/Configs/ActuatorsConfig.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>

#include "ActuatorsMacros.h"

using namespace Boardcore;
using namespace miosix;
using namespace Common;
using namespace RIGv2;

void Actuators::ServoInfo::openServoWithTime(uint32_t time)
{
    long long currentTime = getTime();

    closeTs      = currentTime + (time * Constants::NS_IN_MS);
    lastActionTs = currentTime;

    if (config.openingEvent != 0)
        EventBroker::getInstance().post(config.openingEvent, TOPIC_MOTOR);
}

void Actuators::ServoInfo::closeServo()
{
    closeTs      = 0;
    lastActionTs = getTime();

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
    : infos{
          MAKE_SERVO(OX_FIL),        MAKE_SERVO(OX_REL),
          MAKE_DETACH_SERVO(OX_DET),
          MAKE_SERVO(N2_FIL),        MAKE_SERVO(N2_REL),
          MAKE_DETACH_SERVO(N2_DET), MAKE_SERVO(NITR),
          MAKE_SERVO(OX_VEN),        MAKE_SERVO(N2_QUE),
          MAKE_SERVO(MAIN),
      }, n2_3wayValveInfo(MAKE_SIMPLE_SERVO(N2_3W))
{
    for (auto& servo : infos)
        servo.unsafeSetServoPosition(0.0f);

    n2_3wayValveInfo.unsafeSetServoPosition(0.0f);
}

bool Actuators::isStarted() { return started; }

bool Actuators::start()
{
    TaskScheduler& scheduler =
        getModule<BoardScheduler>()->getActuatorsScheduler();

    // Enable all servos
    for (ServoInfo& info : infos)
        info.servo->enable();

    uint8_t result =
        scheduler.addTask([this]() { updatePositionsTask(); },
                          Config::Servos::SERVO_TIMINGS_CHECK_PERIOD);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add updatePositionsTask");
        return false;
    }

    started = true;
    return true;
}

void Actuators::armLightOn() { relays::armLight::low(); }
void Actuators::armLightOff() { relays::armLight::high(); }

void Actuators::igniterOn() { relays::ignition::low(); }
void Actuators::igniterOff() { relays::ignition::high(); }

bool Actuators::wiggleServo(ServosList servo)
{
    // Wiggle means open the servo for 1s
    return openServoWithTime(servo, 1000);
}

bool Actuators::toggleServo(ServosList servo)
{
    Lock<FastMutex> lock(infosMutex);
    ServoInfo* info = getServo(servo);
    if (info == nullptr)
        return false;

    if (info->closeTs == 0)
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

    return info->closeTs != 0;
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

void Actuators::set3wayValveState(bool state)
{
    auto position = state ? 1.0f : 0.0f;
    n2_3wayValveInfo.unsafeSetServoPosition(position);
}

bool Actuators::get3wayValveState()
{
    return n2_3wayValveInfo.getServoPosition() == 1.0f;
}

void Actuators::openChamberWithTime(uint32_t time)
{
    Lock<FastMutex> lock(infosMutex);
    long long currentTime = getTime();

    chamberCloseTs      = currentTime + (time * Constants::NS_IN_MS);
    chamberLastActionTs = currentTime;
}

void Actuators::closeChamber()
{
    Lock<FastMutex> lock(infosMutex);
    chamberCloseTs = 0;
}

bool Actuators::isChamberOpen()
{
    Lock<FastMutex> lock(infosMutex);
    return chamberCloseTs != 0;
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
            return &infos[4];
        case N2_RELEASE_VALVE:  // N2_REL
            return &infos[5];
        case N2_DETACH_SERVO:  // N2_DET
            return &infos[6];
        case NITROGEN_VALVE:  // NITR
            return &infos[7];
        case OX_VENTING_VALVE:  // OX_VEN
            return &infos[8];
        case N2_QUENCHING_VALVE:  // N2_QUE
            return &infos[9];
        case MAIN_VALVE:  // MAIN
            return &infos[10];

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
            if (currentTime < infos[idx].lastActionTs +
                                  (Config::Servos::SERVO_CONFIDENCE_TIME *
                                   Constants::NS_IN_MS))
            {
                // We should open the valve all the way
                unsafeSetServoPosition(idx, infos[idx].getMaxAperture());
            }
            else
            {
                // Time to wiggle the valve a little
                unsafeSetServoPosition(
                    idx, infos[idx].getMaxAperture() *
                             (1.0 - Config::Servos::SERVO_CONFIDENCE));
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
                unsafeSetServoPosition(idx,
                                       infos[idx].getMaxAperture() *
                                           Config::Servos::SERVO_CONFIDENCE);
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
        chamberCloseTs = 0;

        unsafeCloseChamber();
    }
}
