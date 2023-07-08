/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <utils/ModuleManager/ModuleManager.hpp>

#include "ActuatorsConfig.h"
#include "logger/Logger.h"

using namespace miosix;
using namespace Boardcore;

namespace Antennas
{
// TIM1_CH4 PA11 AF1
//      |
// TIM3_CH2 PC7  AF2

// TIM4_CH1 PD12 AF2
//      |
// TIM8_CH4 PC9  AF3

GpioPin stepPinX      = GpioPin(GPIOD_BASE, 12);  // tim4_ch1
GpioPin countPinX     = GpioPin(GPIOC_BASE, 9);   // tim8_ch4
GpioPin directionPinX = GpioPin(GPIOD_BASE, 13);
GpioPin enablePinX    = GpioPin(GPIOD_BASE, 3);

GpioPin stepPinY      = GpioPin(GPIOA_BASE, 11);  // tim1_ch4
GpioPin countPinY     = GpioPin(GPIOC_BASE, 7);   // tim3_ch2
GpioPin directionPinY = GpioPin(GPIOA_BASE, 12);
GpioPin enablePinY    = GpioPin(GPIOA_BASE, 8);

GpioPin ledRGB = GpioPin(GPIOG_BASE, 14);

CountedPWM countedPwmX(Config::StepperConfig::SERVO2_PULSE_TIM,
                       Config::StepperConfig::SERVO2_PULSE_CH,
                       Config::StepperConfig::SERVO2_PULSE_ITR,
                       Config::StepperConfig::SERVO2_COUNT_TIM,
                       Config::StepperConfig::SERVO2_COUNT_CH,
                       Config::StepperConfig::SERVO2_COUNT_ITR);

CountedPWM countedPwmY(Config::StepperConfig::SERVO1_PULSE_TIM,
                       Config::StepperConfig::SERVO1_PULSE_CH,
                       Config::StepperConfig::SERVO1_PULSE_ITR,
                       Config::StepperConfig::SERVO1_COUNT_TIM,
                       Config::StepperConfig::SERVO1_COUNT_CH,
                       Config::StepperConfig::SERVO1_COUNT_ITR);

Actuators::Actuators()
    : stepperX(countedPwmX, stepPinX, directionPinX, 1, 1.8, false, 4,
               Stepper::PinConfiguration::COMMON_CATHODE, enablePinX),
      stepperY(countedPwmY, stepPinY, directionPinY, 1, 1.8, false, 4,
               Stepper::PinConfiguration::COMMON_CATHODE, enablePinY)
{
    // set LED to Yellow

    stepPinX.mode(Mode::ALTERNATE);
    stepPinX.alternateFunction(2);
    stepPinY.mode(Mode::ALTERNATE);
    stepPinY.alternateFunction(1);

    directionPinX.mode(Mode::OUTPUT);
    enablePinX.mode(Mode::OUTPUT);
    directionPinY.mode(Mode::OUTPUT);
    enablePinY.mode(Mode::OUTPUT);

#ifdef NO_SD_LOGGING
    countPinX.mode(Mode::ALTERNATE);
    countPinX.alternateFunction(3);
    countPinY.mode(Mode::ALTERNATE);
    countPinY.alternateFunction(2);
#endif

    // Set LED to GREEN
}

/**
 * @brief Enables all the stepperPWM
 */
void Actuators::start()
{
    stepperX.enable();
    stepperY.enable();
}

void Actuators::setSpeed(StepperList axis, float speed)
{

    switch (axis)
    {
        case StepperList::HORIZONTAL:
            if (speed > Config::MAX_SPEED_HORIZONTAL)
            {
                speed = Config::MAX_SPEED_HORIZONTAL;
            }
            stepperX.setSpeed(speed);
            break;
        case StepperList::VERTICAL:
            if (speed > Config::MAX_SPEED_VERTICAL)
            {
                speed = Config::MAX_SPEED_VERTICAL;
            }
            stepperY.setSpeed(speed);
            break;
        default:
            assert(false && "Non existent stepper");
            break;
    }
}

int16_t Actuators::getCurrentPosition(StepperList axis)
{
    switch (axis)
    {
        case StepperList::HORIZONTAL:
            return stepperX.getCurrentPosition();
            break;
        case StepperList::VERTICAL:
            return stepperY.getCurrentPosition();
            break;
        default:
            assert(false && "Non existent stepper");
            break;
    }
    return 0;
}

float Actuators::getCurrentDegPosition(StepperList axis)
{
    switch (axis)
    {
        case StepperList::HORIZONTAL:
            return stepperX.getCurrentDegPosition();
            break;
        case StepperList::VERTICAL:
            return stepperY.getCurrentDegPosition();
            break;
        default:
            assert(false && "Non existent stepper");
            break;
    }
    return 0;
}

void Actuators::moveDeg(StepperList axis, float degrees)
{
    if (emergencyStop)
    {
        Logger::getInstance().log(stepperY.getState(0));
        return;
    }

    switch (axis)
    {
        float newDegrees;
        case StepperList::HORIZONTAL:
            // LIMIT POSITION IN ACCEPTABLE RANGE
            newDegrees = stepperX.getCurrentDegPosition() + degrees;
            if (newDegrees > Config::MAX_ANGLE_HORIZONTAL)
            {
                degrees = Config::MAX_ANGLE_HORIZONTAL -
                          stepperX.getCurrentDegPosition();
            }
            else if (newDegrees < Config::MIN_ANGLE_HORIZONTAL)
            {
                degrees = Config::MIN_ANGLE_HORIZONTAL -
                          stepperX.getCurrentDegPosition();
            }

            stepperX.moveDeg(degrees);
            Logger::getInstance().log(stepperX.getState(degrees));
            break;
        case StepperList::VERTICAL:
            // LIMIT POSITION IN ACCEPTABLE RANGE
            newDegrees = stepperY.getCurrentDegPosition() + degrees;
            if (newDegrees > Config::MAX_ANGLE_VERTICAL)
            {
                degrees = Config::MAX_ANGLE_VERTICAL -
                          stepperY.getCurrentDegPosition();
            }
            else if (newDegrees < Config::MIN_ANGLE_VERTICAL)
            {
                degrees = Config::MIN_ANGLE_VERTICAL -
                          stepperY.getCurrentDegPosition();
            }

            stepperY.moveDeg(degrees);
            Logger::getInstance().log(stepperY.getState(degrees));
            break;
        default:
            assert(false && "Non existent stepper");
            break;
    }
}

void Actuators::setPositionDeg(StepperList axis, float positionDeg)
{
    float currentDegPosition;
    switch (axis)
    {
        case StepperList::HORIZONTAL:
            currentDegPosition = stepperX.getCurrentDegPosition();
            break;
        case StepperList::VERTICAL:
            currentDegPosition = stepperY.getCurrentDegPosition();
            break;
        default:
            assert(false && "Non existent stepper");
            return;
    }

    moveDeg(axis, positionDeg - currentDegPosition);
}

void Actuators::IRQemergencyStop()
{
    // Do not preempt during this method
    emergencyStop = true;
    countedPwmX.stop();  // Terminate current stepper actuation
    countedPwmY.stop();  // Terminate current stepper actuation
    stepperX.disable();  // Disable the horizontal movement
    stepperY.enable();   // Don't make the antenna fall

    ledOn();

    // Set LED to RED
}

void Actuators::IRQemergencyStopRecovery()
{
    // Do not preempt during this method
    emergencyStop = false;
    stepperX.enable();  // Re-enable horizontal movement
    stepperY.enable();  // Re-enable vertical movement

    ledOff();

    // Set LED to GREEN
}

}  // namespace Antennas