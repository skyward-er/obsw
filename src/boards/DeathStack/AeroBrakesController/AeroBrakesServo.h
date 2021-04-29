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
using namespace DeathStackBoard::AeroBrakesConfigs;

namespace DeathStackBoard
{
class AeroBrakesServo : public ServoInterface
{
public:
    AeroBrakesServo() : ServoInterface(SERVO_MIN_POS, SERVO_MAX_POS) {}
    AeroBrakesServo(float minPosition, float maxPosition)
        : ServoInterface(minPosition, maxPosition)
    {
    }

    AeroBrakesServo(float minPosition, float maxPosition, float resetPosition)
        : ServoInterface(minPosition, maxPosition, resetPosition)
    {
    }

    virtual ~AeroBrakesServo()
    {
        
    }

    void enable() override
    {
        servo.setMaxPulseWidth(2500);
        servo.setMinPulseWidth(500);
        servo.enable(SERVO_PWM_CH);
        servo.start();
    }

    void disable() override
    {
        servo.stop();
        servo.disable(SERVO_PWM_CH);
    }

    /**
     * @brief Perform wiggle around the middle point.
     */
    void selfTest() override
    {
        float base   = (MAX_POS + RESET_POS) / 2;
        float maxpos = base + SERVO_WIGGLE_AMPLITUDE / 2;
        float minpos = base - SERVO_WIGGLE_AMPLITUDE / 2;

        set(base);

        for (int i = 0; i < 3; i++)
        {
            miosix::Thread::sleep(UPDATE_TIME + 100);
            set(maxpos);
            miosix::Thread::sleep(UPDATE_TIME + 100);
            set(minpos);
        }

        miosix::Thread::sleep(UPDATE_TIME);
    }

private:
    Servo servo{SERVO_TIMER};

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
        servo.setPosition(AeroBrakesConfigs::SERVO_PWM_CH, angle / 180.0f);

        AeroBrakesData abdata;
        abdata.timestamp = miosix::getTick();
        abdata.servo_position =
            servo.getPosition(AeroBrakesConfigs::SERVO_PWM_CH);
        LoggerService::getInstance()->log(abdata);
    }

    float preprocessPosition(float angle) override
    {
        angle = ServoInterface::preprocessPosition(angle);

        float update_time_seconds = UPDATE_TIME / 1000;
        float rate = (angle - currentPosition) / update_time_seconds;

        if (rate > SERVO_MAX_RATE)
        {
            angle = update_time_seconds * SERVO_MAX_RATE + currentPosition;
        }
        else if (rate < SERVO_MIN_RATE)
        {
            angle = update_time_seconds * SERVO_MIN_RATE + currentPosition;
        }

        angle =
            FILTER_COEFF * angle + (1 - FILTER_COEFF) * getCurrentPosition();

        return angle;
    }
};
}  // namespace DeathStackBoard