/*
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include "../ServoInterface.h"
#include "../configs/DeploymentConfig.h"
#include "drivers/servo/servo.h"
#include "miosix.h"

namespace DeathStackBoard
{

using namespace DeathStackBoard::DeploymentConfigs;

class DeploymentServo : public ServoInterface
{
public:
    Servo servo{DPL_SERVO_TIMER};

    DeploymentServo()
        : ServoInterface(DPL_SERVO_MIN_POS, DPL_SERVO_MAX_POS, DPL_SERVO_RESET_POS)
    {
    }

    DeploymentServo(float minPosition, float maxPosition, float resetPosition)
        : ServoInterface(minPosition, maxPosition, resetPosition)
    {
    }

    void enable() override
    {
        servo.setMinPulseWidth(500);
        servo.setMaxPulseWidth(2500);
        servo.enable(DPL_SERVO_PWM_CH);
        servo.start();
    }

    void disable() override
    {
        servo.stop();
        servo.disable(DPL_SERVO_PWM_CH);
    }

    /**
     * @brief Perform wiggle around the reset position.
     */
    void selfTest() override
    {
        for (int i = 0; i < 3; i++)
        {
            set(RESET_POS - DPL_SERVO_WIGGLE_AMPLITUDE);
            miosix::Thread::sleep(500);
            reset();
            miosix::Thread::sleep(500);
        }
    }

protected:
    void setPosition(float angle) override
    {
        currentPosition = angle;
        servo.setPosition(DPL_SERVO_PWM_CH, currentPosition / 180.0f);
    }

private:
    float anglePrec = DPL_SERVO_RESET_POS;
};

}  // namespace DeathStackBoard