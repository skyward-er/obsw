/*
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Vincenzo Santomarco
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
#include "../configs/AeroBrakesConfig.h"
#include "AeroBrakesData.h"
#include "LoggerService/LoggerService.h"
#include "drivers/servo/servo.h"
#include "miosix.h"

#ifdef HARDWARE_IN_THE_LOOP
#include "hardware_in_the_loop/HIL.h"
#endif

namespace DeathStackBoard
{

using namespace DeathStackBoard::AeroBrakesConfigs;

class AeroBrakesServo : public ServoInterface
{
public:
    AeroBrakesServo() : ServoInterface(AB_SERVO_MIN_POS, AB_SERVO_MAX_POS) {}
    AeroBrakesServo(float minPosition, float maxPosition)
        : ServoInterface(minPosition, maxPosition)
    {
    }

    AeroBrakesServo(float minPosition, float maxPosition, float resetPosition)
        : ServoInterface(minPosition, maxPosition, resetPosition)
    {
    }

    virtual ~AeroBrakesServo() {}

    void enable() override
    {
        servo.setMaxPulseWidth(2500);
        servo.setMinPulseWidth(500);
        servo.enable(AB_SERVO_PWM_CH);
        servo.start();
    }

    void disable() override
    {
        servo.stop();
        servo.disable(AB_SERVO_PWM_CH);
    }

    /**
     * @brief Perform wiggle around the middle point.
     */
    void selfTest() override
    {
        float base   = (MAX_POS + RESET_POS) / 2;
        float maxpos = base + AB_SERVO_WIGGLE_AMPLITUDE / 2;
        float minpos = base - AB_SERVO_WIGGLE_AMPLITUDE / 2;

        set(base, true);

        for (int i = 0; i < 3; i++)
        {
            miosix::Thread::sleep(ABK_UPDATE_PERIOD + 100);
            set(maxpos, true);
            miosix::Thread::sleep(ABK_UPDATE_PERIOD + 100);
            set(minpos, true);
        }

        miosix::Thread::sleep(ABK_UPDATE_PERIOD);
        reset();
    }

private:
    Servo servo{AeroBrakesConfigs::AB_SERVO_TIMER};

#ifdef HARDWARE_IN_THE_LOOP
    HIL *simulator = HIL::getInstance();
#endif

protected:
    /**
     * @brief Set servo position.
     *
     * @param angle servo position (in degrees)
     */
    void setPosition(float angle) override
    {
        currentPosition = angle;
        // map position to [0;1] interval for the servo driver
        servo.setPosition(AeroBrakesConfigs::AB_SERVO_PWM_CH, angle / 180.0f);

#ifdef HARDWARE_IN_THE_LOOP
        simulator->send(angle);
#endif
    }

    float preprocessPosition(float angle) override
    {
        angle = ServoInterface::preprocessPosition(angle);
        
        float rate = (angle - currentPosition) / ABK_UPDATE_PERIOD_SECONDS;

        if (rate > AB_SERVO_MAX_RATE)
        {
            angle = ABK_UPDATE_PERIOD_SECONDS * AB_SERVO_MAX_RATE + currentPosition;
        }
        else if (rate < AB_SERVO_MIN_RATE)
        {
            angle = ABK_UPDATE_PERIOD_SECONDS * AB_SERVO_MIN_RATE + currentPosition;
        }

        angle =
            FILTER_COEFF * angle + (1 - FILTER_COEFF) * getCurrentPosition();

        return angle;
    }
};
}  // namespace DeathStackBoard