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

    void emergencyStop();
    void emergencyStopRecovery();

    void setSpeed(StepperList axis, float speed);

    void move(StepperList axis, int16_t steps);
    void moveDeg(StepperList axis, float degrees);
    void setPosition(StepperList axis, int16_t steps);
    void setPositionDeg(StepperList axis, float degrees);

    int16_t getCurrentPosition(StepperList axis);
    float getCurrentDegPosition(StepperList axis);

private:
    Boardcore::StepperPWM& getServo(StepperList servo);

    Boardcore::StepperPWM stepperX;
    Boardcore::StepperPWM stepperY;
    bool stepperXActive = true;  // Whether stepper on X axis can accept steps
    bool stepperYActive = true;  // Whether stepper on Y axis can accept steps
};
}  // namespace Antennas