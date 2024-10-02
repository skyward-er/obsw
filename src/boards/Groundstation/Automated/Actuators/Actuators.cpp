/* Copyright (c) 2023-2024 Skyward Experimental Rocketry
 * Author: Emilio Corigliano, Nicolò Caruso
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

#include <interfaces-impl/hwmapping.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#include "ActuatorsConfig.h"

using namespace miosix;
using namespace Boardcore;

namespace Antennas
{

using namespace Config;

// TIM1_CH4 PA11 AF1
//      |
// TIM3_CH2 PC7  AF2

// TIM4_CH1 PD12 AF2
//      |
// TIM8_CH4 PC9  AF3

GpioPin ledRGB = GpioPin(GPIOG_BASE, 14);

CountedPWM countedPwmX(StepperSettings::SERVO1_PULSE_TIM,
                       StepperSettings::SERVO1_PULSE_CH,
                       StepperSettings::SERVO1_PULSE_ITR,
                       StepperSettings::SERVO1_COUNT_TIM,
                       StepperSettings::SERVO1_COUNT_CH,
                       StepperSettings::SERVO1_COUNT_ITR);

CountedPWM countedPwmY(StepperSettings::SERVO2_PULSE_TIM,
                       StepperSettings::SERVO2_PULSE_CH,
                       StepperSettings::SERVO2_PULSE_ITR,
                       StepperSettings::SERVO2_COUNT_TIM,
                       StepperSettings::SERVO2_COUNT_CH,
                       StepperSettings::SERVO2_COUNT_ITR);

Actuators::Actuators()
    : stepperX(countedPwmX, stepper1::pulseTimer::getPin(),
               stepper1::direction::getPin(), stepperXConfig.MAX_SPEED,
               stepperXConfig.STEP_ANGLE, false, stepperXConfig.MICROSTEPPING,
               Stepper::PinConfiguration::COMMON_CATHODE,
               stepper1::enable::getPin()),
      stepperY(countedPwmY, stepper2::pulseTimer::getPin(),
               stepper2::direction::getPin(), stepperYConfig.MAX_SPEED,
               stepperYConfig.STEP_ANGLE, true, stepperYConfig.MICROSTEPPING,
               Stepper::PinConfiguration::COMMON_CATHODE,
               stepper2::enable::getPin())
{
}

/**
 * @brief Dummy start for actuators
 * @note The real enable is done by the `arm()` method
 */
void Actuators::start() {}

void Actuators::arm()
{
    stepperX.enable();
    stepperY.enable();
}

void Actuators::disarm()
{
    stepperX.disable();
    stepperY.disable();
}

void Actuators::setSpeed(StepperList axis, float speed)
{
    const auto *config = getStepperConfig(axis);
    auto *stepper      = getStepper(axis);

    if (speed > config->MAX_SPEED)
    {
        speed = config->MAX_SPEED;
    }
    stepper->setSpeed(speed);

    switch (axis)
    {
        case StepperList::STEPPER_X:
            speedX = speed;
            break;
        case StepperList::STEPPER_Y:
            speedY = speed;
            break;
        default:
            assert(false && "Non existent stepper");
            break;
    }
}

void Actuators::zeroPosition()
{
    stepperX.zeroPosition();
    stepperY.zeroPosition();
}

int16_t Actuators::getCurrentPosition(StepperList axis)
{
    return getStepper(axis)->getCurrentPosition();
}

float Actuators::getCurrentDegPosition(StepperList axis)
{
    const auto *config = getStepperConfig(axis);
    auto *stepper      = getStepper(axis);

    return stepper->getCurrentDegPosition() / config->MULTIPLIER;
}

ErrorMovement Actuators::move(StepperList axis, int16_t steps)
{
    auto *stepper = getStepper(axis);

    if (emergencyStop)
    {
        logStepperData(axis, stepper->getState(0));
        return ErrorMovement::EMERGENCY_STOP;
    }

    ErrorMovement actuationState =
        ErrorMovement::OK;  //< In case the move command is not limited

    if (!stepper.isEnabled())
        return ErrorMovement::DISABLED;

    const auto *config = getStepperConfig(axis);
    // float position     = getCurrentPosition(axis);

    // int16_t maxSteps =
    //     config->MAX_ANGLE * config->MICROSTEPPING / config->STEP_ANGLE;
    // int16_t minSteps =
    //     config->MIN_ANGLE * config->MICROSTEPPING / config->STEP_ANGLE;

    // // LIMIT POSITION IN ACCEPTABLE RANGE
    // if (position + steps > maxSteps)
    // {
    //     steps = config.MAX_ANGLE - position;
    //     actuationState = ErrorMovement::LIMIT;
    // }
    // else if (position + steps < minSteps)
    // {
    //     steps = config->MIN_ANGLE - position;
    // }

    stepper->move(steps);
    logStepperData(axis, stepper->getState(steps * config->STEP_ANGLE /
                                           config->MICROSTEPPING));
    return actuationState;
}

ErrorMovement Actuators::moveDeg(StepperList axis, float degrees)
{
    auto *stepper = getStepper(axis);

    if (emergencyStop)
    {
        logStepperData(axis, stepper->getState(0));
        return ErrorMovement::EMERGENCY_STOP;
    }

    const auto *config = getStepperConfig(axis);
    // float positionDeg  = getCurrentDegPosition(axis);

    // LIMIT POSITION IN ACCEPTABLE RANGE
    // if (positionDeg + degrees > config->MAX_ANGLE)
    // {
    //     degrees = config->MAX_ANGLE - positionDeg;
    // }
    // else if (positionDeg + degrees < config->MIN_ANGLE)
    // {
    //     degrees = config->MIN_ANGLE - positionDeg;
    // }

    // Moving stepper of the angle 'degrees'
    stepper->moveDeg(degrees * config->MULTIPLIER);
    logStepperData(axis, stepper->getState(degrees));
}

ErrorMovement Actuators::setPositionDeg(StepperList axis, float positionDeg)
{
    return moveDeg(axis, positionDeg - getCurrentDegPosition(axis));
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
