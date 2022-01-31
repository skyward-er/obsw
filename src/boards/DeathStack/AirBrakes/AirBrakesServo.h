/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Vincenzo Santomarco
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

#include <AirBrakes/AirBrakesData.h>
#include <LoggerService/LoggerService.h>
#include <common/ServoInterface.h>
#include <configs/AirBrakesConfig.h>
#include <drivers/servo/Servo.h>
#include <miosix.h>

#ifdef HARDWARE_IN_THE_LOOP
#include <hardware_in_the_loop/HIL.h>
#endif

namespace DeathStackBoard
{

using namespace AirBrakesConfigs;

class AirBrakesServo : public ServoInterface
{
public:
    AirBrakesServo();

    AirBrakesServo(float minPosition, float maxPosition);

    AirBrakesServo(float minPosition, float maxPosition, float resetPosition);

    virtual ~AirBrakesServo();

    void enable() override;

    void disable() override;

    /**
     * @brief Perform wiggle around the middle point.
     */
    void selfTest() override;

private:
    Servo servo{AirBrakesConfigs::AB_SERVO_TIMER, AB_SERVO_PWM_CH, 50, 500,
                2500};

#ifdef HARDWARE_IN_THE_LOOP
    HIL *simulator = HIL::getInstance();
#endif

protected:
    /**
     * @brief Set servo position.
     *
     * @param angle servo position (in degrees)
     */
    void setPosition(float angle) override;

    float preprocessPosition(float angle) override;
};

}  // namespace DeathStackBoard