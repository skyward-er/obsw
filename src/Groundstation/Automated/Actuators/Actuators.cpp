/* Copyright (c) 2023-2024 Skyward Experimental Rocketry
 * Authors: Emilio Corigliano, Nicolò Caruso
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

#include "ActuatorsConfig.h"

using namespace miosix;
using namespace Boardcore;

namespace Antennas
{

using namespace Config;

// TIM1_CH1 PA8 AF1 Stepper H step
//      |
// TIM3_CH2 PC7  AF2 Stepper H count

// TIM4_CH1 PD12 AF2 Stepper V step
//      |
// TIM8_CH1 PC6  AF3 Stepper V count

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

ActuationStatus Actuators::setSpeed(StepperList axis, float speed)
{
    const auto* config  = getStepperConfig(axis);
    auto* stepper       = getStepper(axis);
    auto actuationState = ActuationStatus::OK;
    auto multiplier     = getStepperMultiplier(axis);

    speed *= multiplier;

    if (speed > config->MAX_SPEED)
    {
        speed          = config->MAX_SPEED;
        actuationState = ActuationStatus::SPEED_LIMIT;
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
            actuationState = ActuationStatus::NO_STEPPER;
            break;
    }

    return actuationState;
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
    auto multiplier = getStepperMultiplier(axis);
    auto* stepper   = getStepper(axis);

    return stepper->getCurrentDegPosition() / multiplier;
}

ActuationStatus Actuators::move(StepperList axis, int16_t steps)
{
    auto* stepper = getStepper(axis);

    ActuationStatus actuationState =
        ActuationStatus::OK;  //< In case the move command is not limited

    if (!stepper->isEnabled())
        return ActuationStatus::DISABLED;

    const auto* config = getStepperConfig(axis);
    float position     = getCurrentPosition(axis);

    int16_t maxSteps =
        config->MAX_ANGLE * config->MICROSTEPPING / config->STEP_ANGLE;
    int16_t minSteps =
        config->MIN_ANGLE * config->MICROSTEPPING / config->STEP_ANGLE;

    // POSITION_LIMIT POSITION IN ACCEPTABLE RANGE
    if (position + steps > maxSteps)
    {
        steps          = config->MAX_ANGLE - position;
        actuationState = ActuationStatus::POSITION_LIMIT;
    }
    else if (position + steps < minSteps)
    {
        steps = config->MIN_ANGLE - position;
    }

    stepper->move(steps);
    logStepperData(axis, stepper->getState(steps * config->STEP_ANGLE /
                                           config->MICROSTEPPING));
    return actuationState;
}

ActuationStatus Actuators::moveDeg(StepperList axis, float degrees)
{
    auto* stepper = getStepper(axis);

    ActuationStatus actuationState =
        ActuationStatus::OK;  //< In case the move command is not limited

    const auto* config = getStepperConfig(axis);
    auto multiplier    = getStepperMultiplier(axis);
    float positionDeg  = getCurrentDegPosition(axis);

    // POSITION_LIMIT POSITION IN ACCEPTABLE RANGE
    if (positionDeg + degrees > config->MAX_ANGLE)
    {
        degrees        = config->MAX_ANGLE - positionDeg;
        actuationState = ActuationStatus::POSITION_LIMIT;
    }
    else if (positionDeg + degrees < config->MIN_ANGLE)
    {
        degrees = config->MIN_ANGLE - positionDeg;
    }

    // Moving stepper of the angle 'degrees'
    stepper->moveDeg(degrees * multiplier);
    logStepperData(axis, stepper->getState(degrees));
    return actuationState;
}

ActuationStatus Actuators::setPositionDeg(StepperList axis, float positionDeg)
{
    return moveDeg(axis, positionDeg - getCurrentDegPosition(axis));
}

void Actuators::setMultipliers(StepperList axis, float multiplier)
{
    switch (axis)
    {
        case StepperList::STEPPER_X:
            multiplierX = multiplier;
            break;
        case StepperList::STEPPER_Y:
            multiplierY = multiplier;
            break;
        default:
            assert(false && "Non existent stepper");
            break;
    }
}
}  // namespace Antennas
