/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Alberto Nidasio, Federico Lolli
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

#include <Payload/BoardScheduler.h>
#include <Payload/Configs/ActuatorsConfigs.h>
#include <common/LedConfig.h>
#include <interfaces-impl/bsp_impl.h>
#include <interfaces-impl/hwmapping.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace miosix;
using namespace Boardcore;
using namespace Common;
using namespace Payload::ActuatorsConfigs;

namespace Payload
{

bool Actuators::start()
{
    ModuleManager& modules = ModuleManager::getInstance();
    scheduler              = modules.get<BoardScheduler>()->getScheduler(4);
    return scheduler == nullptr ? false : true;
}

bool Actuators::setServo(ServosList servoId, float percentage)
{
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            leftServo.setPosition(percentage);
            Logger::getInstance().log(leftServo.getState());
            break;
        case PARAFOIL_RIGHT_SERVO:
            rightServo.setPosition(percentage);
            Logger::getInstance().log(rightServo.getState());
            break;
        default:
            return false;
    }

    return true;
}

bool Actuators::setServoAngle(ServosList servoId, float angle)
{
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            leftServo.setPosition(angle / LEFT_SERVO_ROTATION);
            Logger::getInstance().log(leftServo.getState());
            break;
        case PARAFOIL_RIGHT_SERVO:
            rightServo.setPosition(angle / RIGHT_SERVO_ROTATION);
            Logger::getInstance().log(rightServo.getState());
            break;
        default:
            return false;
    }

    return true;
}

bool Actuators::wiggleServo(ServosList servoId)
{
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            leftServo.setPosition(1);
            Logger::getInstance().log(leftServo.getState());
            Thread::sleep(1000);
            leftServo.setPosition(0);
            Logger::getInstance().log(leftServo.getState());
            break;
        case PARAFOIL_RIGHT_SERVO:
            rightServo.setPosition(1);
            Logger::getInstance().log(rightServo.getState());
            Thread::sleep(1000);
            rightServo.setPosition(0);
            Logger::getInstance().log(rightServo.getState());
            break;
        default:
            return false;
    }

    return true;
}

bool Actuators::enableServo(ServosList servoId)
{
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            leftServo.enable();
            break;
        case PARAFOIL_RIGHT_SERVO:
            rightServo.enable();
            break;
        default:
            return false;
    }

    return true;
}

bool Actuators::disableServo(ServosList servoId)
{
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            leftServo.disable();
            break;
        case PARAFOIL_RIGHT_SERVO:
            rightServo.disable();
            break;
        default:
            return false;
    }

    return true;
}

float Actuators::getServoPosition(ServosList servoId)
{

    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            return leftServo.getPosition();
        case PARAFOIL_RIGHT_SERVO:
            return rightServo.getPosition();
        default:
            return 0;
    }

    return 0;
}

float Actuators::getServoAngle(ServosList servoId)
{
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            return leftServo.getPosition() * LEFT_SERVO_ROTATION;
        case PARAFOIL_RIGHT_SERVO:
            return rightServo.getPosition() * RIGHT_SERVO_ROTATION;
        default:
            return 0;
    }

    return 0;
}

void Actuators::cuttersOn() { gpios::cut_trigger::high(); }

void Actuators::cuttersOff() { gpios::cut_trigger::low(); }

void Actuators::camOn() { gpios::camera_enable::high(); }

void Actuators::camOff() { gpios::camera_enable::low(); }

void Actuators::ledArmed()
{
    // disable ledErrorTask if active
    if (blinkState == BlinkState::LED_ERROR)
        scheduler->disableTask(ledErrorTaskId);
    // enable ledArmedTask
    if (ledArmedTaskId != 0)
        scheduler->enableTask(ledArmedTaskId);
    else
        ledArmedTaskId =
            scheduler->addTask([&]() { toggleLed(); }, LED_ARMED_PERIOD);

    blinkState = BlinkState::LED_ARMED;
}

void Actuators::ledDisarmed()
{
    if (blinkState == BlinkState::LED_ARMED)
        scheduler->disableTask(ledArmedTaskId);
    // TODO: check if removal of ledError should be here

    miosix::ledOn();
    ledState   = true;
    blinkState = BlinkState::LED_DISARMED;
}

void Actuators::ledError()
{
    // disable ledArmedTask if active
    if (blinkState == BlinkState::LED_ARMED)
        scheduler->disableTask(ledArmedTaskId);
    // enable ledErrorTask
    if (ledErrorTaskId != 0)
        scheduler->enableTask(ledErrorTaskId);
    else
        ledErrorTaskId =
            scheduler->addTask([&]() { toggleLed(); }, LED_ERROR_PERIOD);

    blinkState = BlinkState::LED_ERROR;
}

void Actuators::ledOff()
{
    if (blinkState == BlinkState::LED_ARMED)
        scheduler->disableTask(ledArmedTaskId);
    if (blinkState == BlinkState::LED_ERROR)
        scheduler->disableTask(ledErrorTaskId);

    ledState = false;
    miosix::ledOff();
    blinkState = BlinkState::LED_OFF;
}

Actuators::Actuators()
    : leftServo(SERVO_1_TIMER, SERVO_1_PWM_CH, LEFT_SERVO_MIN_PULSE,
                LEFT_SERVO_MAX_PULSE),
      rightServo(SERVO_2_TIMER, SERVO_2_PWM_CH, RIGHT_SERVO_MIN_PULSE,
                 RIGHT_SERVO_MAX_PULSE)
{
}

void Actuators::toggleLed()
{
    if (ledState)
        miosix::ledOff();
    else
        miosix::ledOn();

    ledState = !ledState;
}

}  // namespace Payload
