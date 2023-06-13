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

GpioPin stepPin1      = GpioPin(GPIOA_BASE, 11);  // tim1_ch4
GpioPin countPin1     = GpioPin(GPIOC_BASE, 7);   // tim3_ch2
GpioPin directionPin1 = GpioPin(GPIOA_BASE, 12);
GpioPin enablePin1    = GpioPin(GPIOA_BASE, 8);

GpioPin stepPin2      = GpioPin(GPIOD_BASE, 12);  // tim4_ch1
GpioPin countPin2     = GpioPin(GPIOC_BASE, 9);   // tim8_ch4
GpioPin directionPin2 = GpioPin(GPIOD_BASE, 13);
GpioPin enablePin2    = GpioPin(GPIOD_BASE, 3);

GpioPin ledRGB = GpioPin(GPIOG_BASE, 14);

CountedPWM countedPwmX(Config::StepperConfig::SERVO1_PULSE_TIM,
                       Config::StepperConfig::SERVO1_PULSE_CH,
                       Config::StepperConfig::SERVO1_PULSE_ITR,
                       Config::StepperConfig::SERVO1_COUNT_TIM,
                       Config::StepperConfig::SERVO1_COUNT_CH,
                       Config::StepperConfig::SERVO1_COUNT_ITR);

CountedPWM countedPwmY(Config::StepperConfig::SERVO2_PULSE_TIM,
                       Config::StepperConfig::SERVO2_PULSE_CH,
                       Config::StepperConfig::SERVO2_PULSE_ITR,
                       Config::StepperConfig::SERVO2_COUNT_TIM,
                       Config::StepperConfig::SERVO2_COUNT_CH,
                       Config::StepperConfig::SERVO2_COUNT_ITR);

Actuators::Actuators()
    : stepperX(countedPwmX, stepPin1, directionPin1, 1, 1.8, false, 8,
               Stepper::PinConfiguration::COMMON_CATHODE, enablePin1),
      stepperY(countedPwmY, stepPin2, directionPin2, 1, 1.8, false, 8,
               Stepper::PinConfiguration::COMMON_CATHODE, enablePin2)
{
    stepPin1.mode(Mode::ALTERNATE);
    stepPin1.alternateFunction(1);
    stepPin2.mode(Mode::ALTERNATE);
    stepPin2.alternateFunction(2);

    directionPin1.mode(Mode::OUTPUT);
    enablePin1.mode(Mode::OUTPUT);
    directionPin2.mode(Mode::OUTPUT);
    enablePin2.mode(Mode::OUTPUT);
#ifdef NO_SD_LOGGING
    countPin1.mode(Mode::ALTERNATE);
    countPin1.alternateFunction(2);
    countPin2.mode(Mode::ALTERNATE);
    countPin2.alternateFunction(3);
#endif
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
            stepperX.setSpeed(speed);
            break;
        case StepperList::VERTICAL:
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
    switch (axis)
    {
        case StepperList::HORIZONTAL:
            if (stepperXActive)  // Check for emergency stop
            {
                stepperX.moveDeg(degrees);
                Logger::getInstance().log(stepperX.getState(degrees));
            }
            break;
        case StepperList::VERTICAL:
            if (stepperYActive)  // Check for emergency stop
            {
                stepperY.moveDeg(degrees);
                Logger::getInstance().log(stepperY.getState(degrees));
            }
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

void Actuators::emergencyStop()
{
    // Do not preempt during this method
    PauseKernelLock pkLock;
    stepperXActive = false;  // Disable actuation of horizontal stepper
    stepperYActive = false;  // Disable actuation of vertical stepper
    stepperX.disable();      // Disable the horizontal movement
    stepperY.enable();       // Don't make the antenna fall
    countedPwmX.stop();      // Terminate current stepper actuation
    countedPwmY.stop();      // Terminate current stepper actuation
}

void Actuators::emergencyStopRecovery()
{
    // Do not preempt during this method
    PauseKernelLock pkLock;
    stepperXActive = true;  // Re-enable actuation of horizontal stepper
    stepperYActive = true;  // Re-enable actuation of vertical stepper
    stepperX.enable();      // Re-enable horizontal movement
    stepperY.enable();      // Re-enable vertical movement
}

}  // namespace Antennas