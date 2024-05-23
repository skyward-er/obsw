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

// #include <interfaces-impl/hwmapping.h>

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

// GpioPin ledRGB = GpioPin(GPIOG_BASE, 14);

// CountedPWM countedPwmX(Config::StepperConfig::SERVO1_PULSE_TIM,
//                        Config::StepperConfig::SERVO1_PULSE_CH,
//                        Config::StepperConfig::SERVO1_PULSE_ITR,
//                        Config::StepperConfig::SERVO1_COUNT_TIM,
//                        Config::StepperConfig::SERVO1_COUNT_CH,
//                        Config::StepperConfig::SERVO1_COUNT_ITR);

// CountedPWM countedPwmY(Config::StepperConfig::SERVO2_PULSE_TIM,
//                        Config::StepperConfig::SERVO2_PULSE_CH,
//                        Config::StepperConfig::SERVO2_PULSE_ITR,
//                        Config::StepperConfig::SERVO2_COUNT_TIM,
//                        Config::StepperConfig::SERVO2_COUNT_CH,
//                        Config::StepperConfig::SERVO2_COUNT_ITR);

Actuators::Actuators()
//     : stepperX(countedPwmX, stepper1::pulseTimer::getPin(),
//                stepper1::direction::getPin(), Config::MAX_SPEED_HORIZONTAL,
//                Config::HORIZONTAL_STEP_ANGLE, false,
//                Config::HORIZONTAL_MICROSTEPPING,
//                Stepper::PinConfiguration::COMMON_CATHODE,
//                stepper1::enable::getPin()),
//       stepperY(
//           countedPwmY, stepper2::pulseTimer::getPin(),
//           stepper2::direction::getPin(), Config::MAX_SPEED_VERTICAL,
//           Config::VERTICAL_STEP_ANGLE, true, Config::VERTICAL_MICROSTEPPING,
//           Stepper::PinConfiguration::COMMON_CATHODE,
//           stepper2::enable::getPin())
{
}

/**
 * @brief Dummy start for actuators
 * @note The real enable is done by the `arm()` method
 */
void Actuators::start() {}

void Actuators::arm()
{
    // stepperX.enable();
    // stepperY.enable();
}

void Actuators::disarm()
{
    stepperX.disable();
    stepperY.disable();
}

void Actuators::setSpeed(StepperList axis, float speed)
{

    switch (axis)
    {
        case StepperList::STEPPER_X:
            if (speed > Config::MAX_SPEED_HORIZONTAL)
            {
                speed = Config::MAX_SPEED_HORIZONTAL;
            }
            // stepperX.setSpeed(speed);
            speedX = speed;
            break;
        case StepperList::STEPPER_Y:
            if (speed > Config::MAX_SPEED_VERTICAL)
            {
                speed = Config::MAX_SPEED_VERTICAL;
            }
            // stepperY.setSpeed(speed);
            speedY = speed;
            break;
        default:
            assert(false && "Non existent stepper");
            break;
    }
}

void Actuators::zeroPosition()
{
    // stepperX.zeroPosition();
    // stepperY.zeroPosition();
}

int16_t Actuators::getCurrentPosition(StepperList axis)
{
    switch (axis)
    {
        case StepperList::STEPPER_X:
            return positionX;
            break;
        case StepperList::STEPPER_Y:
            return positionY;
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
        case StepperList::STEPPER_X:
            return positionDegX / Config::HORIZONTAL_MULTIPLIER;
            break;
        case StepperList::STEPPER_Y:
            return positionDegY / Config::VERTICAL_MULTIPLIER;
            break;
        default:
            assert(false && "Non existent stepper");
            break;
    }
    return 0;
}

ErrorMovement Actuators::move(StepperList axis, int16_t steps)
{
    float microstepping = 1.0;
    float step_angle    = 1.8;
    switch (axis)
    {
        case StepperList::STEPPER_X:
            microstepping =
                static_cast<float>(Config::HORIZONTAL_MICROSTEPPING);
            step_angle = Config::HORIZONTAL_STEP_ANGLE;
            break;
        case StepperList::STEPPER_Y:
            microstepping = static_cast<float>(Config::VERTICAL_MICROSTEPPING);
            step_angle    = Config::VERTICAL_STEP_ANGLE;
            break;
        default:
            assert(false && "Non existent stepper");
            break;
    }
    return moveDeg(axis,
                   static_cast<float>(steps) * step_angle / microstepping);
}

ErrorMovement Actuators::moveDeg(StepperList axis, float degrees)
{
    if (emergencyStop)
    {
        switch (axis)
        {
            case StepperList::STEPPER_X:
                // Logger::getInstance().log(
                //     static_cast<StepperXData>(stepperX.getState(0)));
                break;
            case StepperList::STEPPER_Y:
                // Logger::getInstance().log(
                //     static_cast<StepperYData>(stepperY.getState(0)));
                break;
            default:
                assert(false && "Non existent stepper");
                break;
        }

        return ErrorMovement::EMERGENCY_STOP;
    }

    ErrorMovement actuationState =
        ErrorMovement::OK;  //< In case the move command is not limited
    float positionDeg = getCurrentDegPosition(axis);
    switch (axis)
    {
        case StepperList::STEPPER_X:

            // if (!stepperX.isEnabled())
            return ErrorMovement::DISABLED;

            // LIMIT POSITION IN ACCEPTABLE RANGE
            if (positionDeg + degrees > Config::MAX_ANGLE_HORIZONTAL)
            {
                degrees        = Config::MAX_ANGLE_HORIZONTAL - positionDeg;
                actuationState = ErrorMovement::LIMIT;
            }
            else if (positionDeg + degrees < Config::MIN_ANGLE_HORIZONTAL)
            {
                degrees = Config::MIN_ANGLE_HORIZONTAL - positionDeg;
            }

            positionDegX += degrees * Config::HORIZONTAL_MULTIPLIER;
            // stepperX.moveDeg(degrees * Config::HORIZONTAL_MULTIPLIER);
            // Logger::getInstance().log(
            //     static_cast<StepperXData>(stepperX.getState(degrees)));
            deltaX = degrees;
            break;
        case StepperList::STEPPER_Y:

            // if (!stepperY.isEnabled())
            return ErrorMovement::DISABLED;

            // LIMIT POSITION IN ACCEPTABLE RANGE
            if (positionDeg + degrees > Config::MAX_ANGLE_VERTICAL)
            {
                degrees        = Config::MAX_ANGLE_VERTICAL - positionDeg;
                actuationState = ErrorMovement::LIMIT;
            }
            else if (positionDeg + degrees < Config::MIN_ANGLE_VERTICAL)
            {
                degrees = Config::MIN_ANGLE_VERTICAL - positionDeg;
            }

            positionDegY += degrees * Config::HORIZONTAL_MULTIPLIER;
            // stepperY.moveDeg(degrees * Config::VERTICAL_MULTIPLIER);
            // Logger::getInstance().log(
            //     static_cast<StepperYData>(stepperY.getState(degrees)));
            deltaY = degrees;
            break;
        default:
            assert(false && "Non existent stepper");
            actuationState = ErrorMovement::NO_STEPPER;
            break;
    }
    return actuationState;
}

ErrorMovement Actuators::setPosition(StepperList axis, int16_t steps)
{

    float microstepping = 1.0;
    float step_angle    = 1.8;
    switch (axis)
    {
        case StepperList::STEPPER_X:
            // if (!stepperX.isEnabled())
            return ErrorMovement::DISABLED;
            microstepping =
                static_cast<float>(Config::HORIZONTAL_MICROSTEPPING);
            step_angle = Config::HORIZONTAL_STEP_ANGLE;
            break;
        case StepperList::STEPPER_Y:
            // if (!stepperY.isEnabled())
            return ErrorMovement::DISABLED;
            microstepping = static_cast<float>(Config::VERTICAL_MICROSTEPPING);
            step_angle    = Config::VERTICAL_STEP_ANGLE;
            break;
        default:
            assert(false && "Non existent stepper");
            return ErrorMovement::NO_STEPPER;
            break;
    }
    return setPositionDeg(
        axis, static_cast<float>(steps) * step_angle / microstepping);
}

ErrorMovement Actuators::setPositionDeg(StepperList axis, float positionDeg)
{
    return moveDeg(axis, positionDeg - getCurrentDegPosition(axis));
}

void Actuators::IRQemergencyStop()
{
    // Do not preempt during this method
    emergencyStop = true;
    // countedPwmX.stop();  // Terminate current stepper actuation
    // countedPwmY.stop();  // Terminate current stepper actuation
    // stepperX.disable();  // Disable the horizontal movement
    // stepperY.enable();   // Don't make the antenna fall

    // ledOn();

    // Set LED to RED
}

void Actuators::IRQemergencyStopRecovery()
{
    // Do not preempt during this method
    emergencyStop = false;
    // stepperX.enable();  // Re-enable horizontal movement
    // stepperY.enable();  // Re-enable vertical movement

    // ledOff();

    // Set LED to GREEN
}

}  // namespace Antennas
