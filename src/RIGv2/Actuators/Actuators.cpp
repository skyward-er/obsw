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
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

using namespace Boardcore;
using namespace miosix;
using namespace Common;
using namespace RIGv2;
using namespace std::chrono;

void Actuators::ServoInfo::openServoWithTime(uint32_t time)
{
    long long currentTime = getTime();

    closeTs  = currentTime + (time * Constants::NS_IN_MS);
    adjustTs = currentTime +
               (Config::Servos::SERVO_CONFIDENCE_TIME * Constants::NS_IN_MS);

    if (openingEvent != 0)
        EventBroker::getInstance().post(openingEvent, TOPIC_MOTOR);

    unsafeSetServoPosition(getMaxAperture());
}

void Actuators::ServoInfo::closeServo()
{
    long long currentTime = getTime();

    closeTs  = 0;
    adjustTs = currentTime +
               (Config::Servos::SERVO_CONFIDENCE_TIME * Constants::NS_IN_MS);

    if (closingEvent != 0)
        EventBroker::getInstance().post(closingEvent, TOPIC_MOTOR);

    unsafeSetServoPosition(0.0);
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

float Actuators::ServoInfo::getMaxAperture()
{
    return getModule<Registry>()->getOrSetDefaultUnsafe(maxApertureKey,
                                                        defaultMaxAperture);
}

uint32_t Actuators::ServoInfo::getOpeningTime()
{
    return getModule<Registry>()->getOrSetDefaultUnsafe(openingTimeKey,
                                                        defaultOpeningTime);
}

bool Actuators::ServoInfo::setMaxAperture(float aperture)
{
    if (aperture >= 0.0 && aperture <= 1.0)
    {
        getModule<Registry>()->setUnsafe(maxApertureKey, aperture);
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
    getModule<Registry>()->setUnsafe(openingTimeKey, time);
    return true;
}

Actuators::Actuators()
    : valveScheduler(STACK_DEFAULT_FOR_PTHREAD,
                     (miosix::Priority)(PRIORITY_MAX - 1))
{
    // Initialize servos
    infos[0].servo = std::make_unique<Servo>(
        MIOSIX_SERVOS_1_TIM, TimerUtils::Channel::MIOSIX_SERVOS_1_CHANNEL,
        Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE,
        Config::Servos::FREQUENCY);
    infos[1].servo = std::make_unique<Servo>(
        MIOSIX_SERVOS_2_TIM, TimerUtils::Channel::MIOSIX_SERVOS_2_CHANNEL,
        Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE,
        Config::Servos::FREQUENCY);
    infos[2].servo = std::make_unique<Servo>(
        MIOSIX_SERVOS_3_TIM, TimerUtils::Channel::MIOSIX_SERVOS_3_CHANNEL,
        Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE,
        Config::Servos::FREQUENCY);
    infos[3].servo = std::make_unique<Servo>(
        MIOSIX_SERVOS_4_TIM, TimerUtils::Channel::MIOSIX_SERVOS_4_CHANNEL,
        Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE,
        Config::Servos::FREQUENCY);
    // This servo is currently unusable, due to it sharing the same timer as
    // miosix, TIM5
    // infos[4].servo = std::make_unique<Servo>(
    //     MIOSIX_SERVOS_5_TIM, TimerUtils::Channel::MIOSIX_SERVOS_5_CHANNEL,
    //     Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE,
    //     Config::Servos::FREQUENCY);
    infos[5].servo = std::make_unique<Servo>(
        MIOSIX_SERVOS_7_TIM, TimerUtils::Channel::MIOSIX_SERVOS_7_CHANNEL,
        Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE,
        Config::Servos::FREQUENCY);
    infos[6].servo = std::make_unique<Servo>(
        MIOSIX_SERVOS_9_TIM, TimerUtils::Channel::MIOSIX_SERVOS_9_CHANNEL,
        Config::Servos::SERVO2_MIN_PULSE, Config::Servos::SERVO2_MAX_PULSE,
        Config::Servos::FREQUENCY);
    // This servo is currently unusable, due to it sharing the same timer as
    // servo 1
    // infos[7].servo = std::make_unique<Servo>(
    //     MIOSIX_SERVOS_8_TIM, TimerUtils::Channel::MIOSIX_SERVOS_8_CHANNEL,
    //     Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE,
    //     Config::Servos::FREQUENCY);
    infos[8].servo = std::make_unique<Servo>(
        MIOSIX_SERVOS_6_TIM, TimerUtils::Channel::MIOSIX_SERVOS_6_CHANNEL,
        Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE,
        Config::Servos::FREQUENCY);
    infos[9].servo = std::make_unique<Servo>(
        MIOSIX_SERVOS_8_TIM, TimerUtils::Channel::MIOSIX_SERVOS_8_CHANNEL,
        Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE,
        Config::Servos::FREQUENCY);

    ServoInfo* info;
    info                     = getServo(ServosList::N2O_FILLING_VALVE);
    info->defaultMaxAperture = Config::Servos::DEFAULT_FILLING_MAX_APERTURE;
    info->defaultOpeningTime = Config::Servos::DEFAULT_FILLING_OPENING_TIME;
    info->limit              = Config::Servos::FILLING_LIMIT;
    info->flipped            = Config::Servos::FILLING_FLIPPED;
    info->openingEvent       = Common::Events::MOTOR_OPEN_FILLING_VALVE;
    info->closingEvent       = Common::Events::MOTOR_CLOSE_FILLING_VALVE;
    info->openingTimeKey     = CONFIG_ID_N2O_FILLING_OPENING_TIME;
    info->maxApertureKey     = CONFIG_ID_N2O_FILLING_MAX_APERTURE;
    info->unsafeSetServoPosition(0.0f);

    info                     = getServo(ServosList::N2O_RELEASE_VALVE);
    info->defaultMaxAperture = Config::Servos::DEFAULT_RELEASE_MAX_APERTURE;
    info->defaultOpeningTime = Config::Servos::DEFAULT_RELEASE_OPENING_TIME;
    info->limit              = Config::Servos::RELEASE_LIMIT;
    info->flipped            = Config::Servos::RELEASE_FLIPPED;
    info->openingEvent       = Common::Events::MOTOR_OPEN_RELEASE_VALVE;
    info->closingEvent       = Common::Events::MOTOR_CLOSE_RELEASE_VALVE;
    info->openingTimeKey     = CONFIG_ID_N2O_RELEASE_OPENING_TIME;
    info->maxApertureKey     = CONFIG_ID_N2O_RELEASE_MAX_APERTURE;
    info->unsafeSetServoPosition(0.0f);

    info                     = getServo(ServosList::N2O_VENTING_VALVE);
    info->defaultMaxAperture = Config::Servos::DEFAULT_VENTING_MAX_APERTURE;
    info->defaultOpeningTime = Config::Servos::DEFAULT_VENTING_OPENING_TIME;
    info->limit              = Config::Servos::VENTING_LIMIT;
    info->flipped            = Config::Servos::VENTING_FLIPPED;
    info->openingEvent       = Common::Events::MOTOR_OPEN_VENTING_VALVE;
    info->closingEvent       = Common::Events::MOTOR_CLOSE_VENTING_VALVE;
    info->openingTimeKey     = CONFIG_ID_N2O_VENTING_OPENING_TIME;
    info->maxApertureKey     = CONFIG_ID_N2O_VENTING_MAX_APERTURE;
    info->unsafeSetServoPosition(0.0f);

    info                     = getServo(ServosList::N2_FILLING_VALVE);
    info->defaultMaxAperture = Config::Servos::DEFAULT_FILLING_MAX_APERTURE;
    info->defaultOpeningTime = Config::Servos::DEFAULT_FILLING_OPENING_TIME;
    info->limit              = Config::Servos::FILLING_LIMIT;
    info->flipped            = Config::Servos::FILLING_FLIPPED;
    info->openingEvent       = Common::Events::MOTOR_OPEN_FILLING_VALVE;
    info->closingEvent       = Common::Events::MOTOR_CLOSE_FILLING_VALVE;
    info->openingTimeKey     = CONFIG_ID_N2_FILLING_OPENING_TIME;
    info->maxApertureKey     = CONFIG_ID_N2_FILLING_MAX_APERTURE;
    info->unsafeSetServoPosition(0.0f);

    info                     = getServo(ServosList::N2_RELEASE_VALVE);
    info->defaultMaxAperture = Config::Servos::DEFAULT_RELEASE_MAX_APERTURE;
    info->defaultOpeningTime = Config::Servos::DEFAULT_RELEASE_OPENING_TIME;
    info->limit              = Config::Servos::RELEASE_LIMIT;
    info->flipped            = Config::Servos::RELEASE_FLIPPED;
    info->openingEvent       = Common::Events::MOTOR_OPEN_RELEASE_VALVE;
    info->closingEvent       = Common::Events::MOTOR_CLOSE_RELEASE_VALVE;
    info->openingTimeKey     = CONFIG_ID_N2_RELEASE_OPENING_TIME;
    info->maxApertureKey     = CONFIG_ID_N2_RELEASE_MAX_APERTURE;
    info->unsafeSetServoPosition(0.0f);

    info                     = getServo(ServosList::N2_DETACH_SERVO);
    info->defaultMaxAperture = Config::Servos::DEFAULT_DISCONNECT_MAX_APERTURE;
    info->defaultOpeningTime = Config::Servos::DEFAULT_DISCONNECT_OPENING_TIME;
    info->limit              = Config::Servos::DISCONNECT_LIMIT;
    info->flipped            = Config::Servos::DISCONNECT_FLIPPED;
    info->openingEvent       = Common::Events::MOTOR_DISCONNECT;
    info->openingTimeKey     = CONFIG_ID_N2_DETACH_OPENING_TIME;
    info->maxApertureKey     = CONFIG_ID_N2_DETACH_MAX_APERTURE;
    info->unsafeSetServoPosition(0.0f);

    info                     = getServo(ServosList::MAIN_VALVE);
    info->defaultMaxAperture = Config::Servos::DEFAULT_MAIN_MAX_APERTURE;
    info->defaultOpeningTime = Config::Servos::DEFAULT_MAIN_OPENING_TIME;
    info->limit              = Config::Servos::MAIN_LIMIT;
    info->flipped            = Config::Servos::MAIN_FLIPPED;
    info->openingEvent       = Common::Events::MOTOR_OPEN_FEED_VALVE;
    info->closingEvent       = Common::Events::MOTOR_CLOSE_FEED_VALVE;
    info->openingTimeKey     = CONFIG_ID_MAIN_OPENING_TIME;
    info->maxApertureKey     = CONFIG_ID_MAIN_MAX_APERTURE;
    info->unsafeSetServoPosition(0.0f);

    info                     = getServo(ServosList::NITROGEN_VALVE);
    info->defaultMaxAperture = Config::Servos::DEFAULT_MAIN_MAX_APERTURE;
    info->defaultOpeningTime = Config::Servos::DEFAULT_MAIN_OPENING_TIME;
    info->limit              = Config::Servos::MAIN_LIMIT;
    info->flipped            = Config::Servos::MAIN_FLIPPED;
    info->openingEvent       = 0;
    info->closingEvent       = 0;
    info->openingTimeKey     = CONFIG_ID_NITROGEN_OPENING_TIME;
    info->maxApertureKey     = CONFIG_ID_NITROGEN_MAX_APERTURE;
    info->unsafeSetServoPosition(0.0f);
}

bool Actuators::isStarted() { return started; }

bool Actuators::start()
{
    TaskScheduler& scheduler =
        getModule<BoardScheduler>()->getActuatorsScheduler();

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

    // Start the ValveScheduler thread
    valveScheduler::start();

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

    cv.notify_one();
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

    cv.notify_one();
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

    cv.notify_one();
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

    cv.notify_one();
    return true;
}

void Actuators::closeAllServos()
{
    Lock<FastMutex> lock(infosMutex);
    for (uint8_t idx = 0; idx < 10; idx++)
        infos[idx].closeServo();

    getModule<CanHandler>()->sendServoCloseCommand(ServosList::MAIN_VALVE);
    getModule<CanHandler>()->sendServoCloseCommand(
        ServosList::N2O_VENTING_VALVE);

    cv.notify_one();
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
    else if (servo == ServosList::N2O_VENTING_VALVE)
        return canVentingOpen;
    else
        return false;
}

void Actuators::openChamberWithTime(uint32_t time)
{
    Lock<FastMutex> lock(infosMutex);
    long long currentTime = getTime();

    nitrogenCloseTs = currentTime + (time * Constants::NS_IN_MS);
    unsafeOpenChamber();
}

void Actuators::closeChamber()
{
    Lock<FastMutex> lock(infosMutex);
    nitrogenCloseTs = 0;
    unsafeCloseChamber();

    cv.notify_one();
}

bool Actuators::isChamberOpen()
{
    Lock<FastMutex> lock(infosMutex);
    return nitrogenCloseTs != 0;
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
    else if (servo == ServosList::N2O_VENTING_VALVE)
        canVentingOpen = open;
}

void Actuators::inject(DependencyInjector& injector)
{
    Super::inject(injector);
    for (ServoInfo& info : infos)
        info.inject(injector);
}

Actuators::ServoInfo* Actuators::getServo(ServosList servo)
{
    // info[4] and info[7] are currently unavailable
    switch (servo)
    {
        case N2O_FILLING_VALVE:
            return &infos[0];
        case N2O_RELEASE_VALVE:
            return &infos[1];
        case N2O_VENTING_VALVE:
            return &infos[2];
        case N2_FILLING_VALVE:
            return &infos[3];
        case N2_RELEASE_VALVE:
            return &infos[5];
        case N2_DETACH_SERVO:
            return &infos[6];
        case MAIN_VALVE:
            return &infos[8];
        case NITROGEN_VALVE:
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

void Actuators::updatePositions()
{
    long long currentTime = getTime();

    {
        Lock<FastMutex> lock(infosMutex);

        // Iterate over all servos
        for (uint8_t idx = 0; idx < 10; idx++)
        {
            if (currentTime < infos[idx].closeTs)
            {
                // The valve should be open
                if (currentTime >= infos[idx].adjustTs)
                {
                    // Time to wiggle the valve a little
                    unsafeSetServoPosition(
                        idx, infos[idx].getMaxAperture() *
                                 (1.0 - Config::Servos::SERVO_CONFIDENCE));

                    infos[idx].adjustTs = 0;
                }
            }
            else
            {
                // Ok the valve is not already closed
                if (infos[idx].closeTs != 0)
                {
                    // Perform the servo closing
                    infos[idx].closeServo();
                }

                if (currentTime < infos[idx].adjustTs)
                {
                    // We should close the valve all the way
                    unsafeSetServoPosition(idx, 0.0);
                }
                else if (infos[idx].adjustTs != 0)
                {
                    // Time to wiggle the valve a little
                    unsafeSetServoPosition(
                        idx, infos[idx].getMaxAperture() *
                                 Config::Servos::SERVO_CONFIDENCE);

                    // The valve has been wiggled
                    infos[idx].adjustTs = 0;
                }
            }
        }
    }

    // Handle nitrogen logic
    if (currentTime > nitrogenCloseTs)
    {
        nitrogenCloseTs = 0;
        unsafeCloseChamber();
    }
}

void Actuators::updateNextActionTs()
{
    Lock<FastMutex> lock(infosMutex);

    nextActionTs = std::numeric_limits<long long>::max();

    // Iterate over all servos and get the smallest (non zero) timestamp
    for (uint8_t idx = 0; idx < 10; idx++)
    {
        if (infos[idx].closeTs != 0 && infos[idx].closeTs < nextActionTs)
            nextActionTs = infos[idx].closeTs;

        if (infos[idx].adjustTs != 0 && infos[idx].adjustTs < nextActionTs)
            nextActionTs = infos[idx].adjustTs;
    }

    if (nitrogenCloseTs != 0 && nitrogenCloseTs < nextActionTs)
        nextActionTs = nitrogenCloseTs;

    if (nextActionTs == std::numeric_limits<long long>::max())
        nextActionTs = 0;
}
