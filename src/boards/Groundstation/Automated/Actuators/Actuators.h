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
#pragma once

#include <utils/ModuleManager/ModuleManager.hpp>

#include "ActuatorsData.h"
#include "actuators/stepper/StepperPWM.h"

// TIM1_CH4 PA11 AF1
//      |
// TIM3_CH2 PC7  AF2

// TIM4_CH1 PD12 AF2
//      |
// TIM8_CH4 PC9  AF3

namespace Antennas
{
class Actuators : public Boardcore::Module
{
public:
    enum StepperList
    {
        HORIZONTAL,  // Stepper moving horizontally (x axis)
        VERTICAL     // Stepper moving vertically (y axis)
    };

    Actuators();

    /**
     * @brief Enables all the servos PWMs
     */
    void start();

    void IRQemergencyStop();
    void IRQemergencyStopRecovery();

    void setSpeed(StepperList axis, float speed);

    void move(StepperList axis, int16_t steps);
    void moveDeg(StepperList axis, float degrees);
    void setPosition(StepperList axis, int16_t steps);
    void setPositionDeg(StepperList axis, float degrees);

    void zeroPosition();

    int16_t getCurrentPosition(StepperList axis);
    float getCurrentDegPosition(StepperList axis);

    bool isEmergencyStopped() { return emergencyStop; }

    /**
     * @brief Getter for the last actuation of the wanted stepper.
     * @param axis The stepper from which we want this information.
     * @returns The last delta angle the chosen stepper is performing [deg].
     */
    float getDeltaAngleDeg(StepperList axis)
    {
        switch (axis)
        {
            case StepperList::HORIZONTAL:
                return deltaX;
            case StepperList::VERTICAL:
                return deltaY;
            default:
                assert(false && "Non existent stepper");
                return 0;
        }
    }

    /**
     * @brief Getter for the speed of the wanted stepper.
     * @param axis The stepper from which we want this information.
     * @returns The speed of the chosen stepper [rps].
     */
    float getSpeed(StepperList axis)
    {
        switch (axis)
        {
            case StepperList::HORIZONTAL:
                return speedX;
            case StepperList::VERTICAL:
                return speedY;
            default:
                assert(false && "Non existent stepper");
                return 0;
        }
    }

private:
    Boardcore::StepperPWM& getServo(StepperList servo);

    Boardcore::StepperPWM stepperX;
    Boardcore::StepperPWM stepperY;

    float deltaX = 0.0f;  // Delta angle to perform [deg]
    float deltaY = 0.0f;  // Delta angle to perform [deg]
    float speedX = 0.0f;  // Speed of the stepper [rps]
    float speedY = 0.0f;  // Speed of the stepper [rps]

    bool emergencyStop =
        false;  // Whether the system performed an emergency stop
};
}  // namespace Antennas