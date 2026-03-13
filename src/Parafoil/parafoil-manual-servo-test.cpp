/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Raul Radu
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

#include <actuators/Servo/ServoWinch.h>
#include <algorithms/SchmittTrigger/SchmittTrigger.h>
#include <miosix.h>
#include <scheduler/TaskScheduler.h>
#include <sensors/AS5047D/AS5047DSPI.h>
#include <units/Angle.h>
#include <utils/ClockUtils.h>

#include <iostream>

#include "test-parafoil-data.h"

using namespace Boardcore;
using namespace Boardcore::Units::Angle;
using namespace miosix;

constexpr Degree SERVO1_ZERO_CALIBRATE = Degree(32.1f);
constexpr Degree SERV02_ZERO_CALIBRATE = Degree(351.2f);

GpioPin sckPin  = GpioPin(GPIOA_BASE, 5);
GpioPin misoPin = GpioPin(GPIOA_BASE, 6);
GpioPin mosiPin = GpioPin(GPIOA_BASE, 7);
GpioPin csPin   = GpioPin(GPIOA_BASE, 15);
GpioPin csPin2  = GpioPin(GPIOB_BASE, 7);

class UnlimitedAngle
{
public:
    UnlimitedAngle() : rounds(0), lastReading(0.0) {}

    void setInitialState(Radian reading) { lastReading = reading; }

    Radian getUpdatedAngle(Radian reading)
    {
        auto angleDiff = reading - lastReading;

        if (angleDiff > threshold)
            rounds--;
        else if (angleDiff < -threshold)
            rounds++;

        lastReading = reading;
        return reading + Radian((Degree(360.0 * rounds)));
    }

private:
    const Radian threshold = Radian(Degree(300.0));
    int rounds;
    Radian lastReading;
};

// Timer 4 CH2
GpioPin pin1(GPIOD_BASE, 13);
// Timer 3 CH1
GpioPin pin2(GPIOC_BASE, 6);

void initBoard()
{
    // Setup gpio pins
    sckPin.mode(Mode::ALTERNATE);
    sckPin.alternateFunction(5);
    misoPin.mode(Mode::ALTERNATE);
    misoPin.alternateFunction(5);
    mosiPin.mode(Mode::ALTERNATE);
    mosiPin.alternateFunction(5);
    csPin.mode(Mode::OUTPUT);
    csPin.high();
    csPin2.mode(Mode::OUTPUT);
    csPin2.high();

    pin1.mode(Mode::ALTERNATE);
    pin1.alternateFunction(2);
    pin2.mode(Mode::ALTERNATE);
    pin2.alternateFunction(2);
}

void triggerIteration() {}

int main()
{
    // Enable SPI clock and set gpios
    initBoard();

    SPIBus spiBus(SPI1);

    ServoWinch s1(TIM4, TimerUtils::Channel::CHANNEL_2, 500_us, 2440_us,
                  333_hz);
    SchmittTrigger trigger(Radian(Degree(10)).value(),
                           Radian(Degree(10)).value());
    AS5047DSPIConfig config;
    config.dataType          = AS5047DDefs::DataSelect::DAECANG;
    config.daecEnabled       = AS5047DDefs::DAECStatus::DAEC_ON;
    config.rotationDirection = AS5047DDefs::ABIRotationDirection::NORMAL;
    AS5047DSPI as5047d(spiBus, csPin, config);
    UnlimitedAngle angleSampler;

    ServoWinch s2(TIM3, TimerUtils::Channel::CHANNEL_1, 500_us, 2440_us,
                  333_hz);
    SchmittTrigger trigger2(Radian(Degree(10)).value(),
                            Radian(Degree(10)).value());
    AS5047DSPIConfig config2;
    config2.dataType          = AS5047DDefs::DataSelect::DAECANG;
    config2.daecEnabled       = AS5047DDefs::DAECStatus::DAEC_ON;
    config2.rotationDirection = AS5047DDefs::ABIRotationDirection::NORMAL;
    AS5047DSPI as5047d2(spiBus, csPin2, config2);
    UnlimitedAngle angleSampler2;

    delayMs(10);
    if (!as5047d.init())
    {
        printf("Failed initialization encoder 1\n");
        for (;;)
        {
        }
    }

    if (!as5047d2.init())
    {
        printf("Failed initialization encoder 2\n");
        for (;;)
        {
        }
    }

    // Enable the timers
    s1.enable();
    s2.enable();

    as5047d.sample();
    delayMs(1);
    as5047d2.sample();

    AS5047DData zeroAngle  = as5047d.getLastSample();
    AS5047DData zeroAngle2 = as5047d2.getLastSample();

    trigger.begin();
    trigger2.begin();
    trigger.setTargetState(Radian(0).value());
    trigger2.setTargetState(Radian(0).value());
    angleSampler.setInitialState(Radian(0));
    angleSampler2.setInitialState(Radian(0));

    printf("Initial reading: T4CH2: %f, %f\r\n",
           Degree(Radian(zeroAngle.angle)).value(),
           Degree(Radian(zeroAngle2.angle)).value());
    printf(
        "timestamp,servo1_raw,servo1,servo1_state,servo2_raw,servo2,servo2_"
        "state\r\n");

    TaskScheduler scheduler;
    scheduler.addTask(
        [&]()
        {
            as5047d.sample();
            AS5047DData data  = as5047d.getLastSample();
            auto angleReading = angleSampler.getUpdatedAngle(
                Radian(data.angle - zeroAngle.angle));
            trigger.setCurrentState(angleReading.value());
            trigger.update();
            SchmittTrigger::Activation output = trigger.getOutput();
            switch (output)
            {
                case SchmittTrigger::Activation::HIGH:
                {
                    s1.setVelocity(1.0);
                    break;
                }
                case SchmittTrigger::Activation::LOW:
                {
                    s1.setVelocity(0.0);
                    break;
                }
                case SchmittTrigger::Activation::STOP:
                {
                    s1.setVelocity(0.5);
                    break;
                }
            }

            as5047d2.sample();
            AS5047DData data2  = as5047d2.getLastSample();
            auto angleReading2 = angleSampler2.getUpdatedAngle(
                Radian(data2.angle - zeroAngle2.angle));

            trigger2.setCurrentState(angleReading2.value());
            trigger2.update();

            SchmittTrigger::Activation output2 = trigger2.getOutput();
            switch (output2)
            {
                case SchmittTrigger::Activation::HIGH:
                {
                    s2.setVelocity(1.0);
                    break;
                }
                case SchmittTrigger::Activation::LOW:
                {
                    s2.setVelocity(0.0);
                    break;
                }
                case SchmittTrigger::Activation::STOP:
                {
                    s2.setVelocity(0.5);
                    break;
                }
            }

            printf("%lld,%f,%f,%f,%d,%f,%f,%f,%d\r\n", data2.timestamp,
                   Degree(Radian(data.angle)).value(),
                   Degree(angleReading).value(),
                   Degree(Radian(trigger.getCurrentTarget())).value(), output,
                   Degree(Radian(data2.angle)).value(),
                   Degree(angleReading2).value(),
                   Degree(Radian(trigger2.getCurrentTarget())).value(),
                   output2);
        },
        10_hz);

    int currAngle = 0;

    scheduler.start();

    // s1.setVelocity(0.5);
    // s2.setVelocity(0.5);
    // Thread::sleep(1000);
    // s1.setVelocity(0.5);
    // s2.setVelocity(0.5);
    // Thread::sleep(1000);

    for (currAngle = 0; currAngle < 100; currAngle++)
    {
        trigger.setTargetState(Radian(Degree(-500)).value());
        trigger2.setTargetState(Radian(Degree(500)).value());

        // s1.setVelocity(0.40);
        // s2.setVelocity(0.40);
        Thread::sleep(2000);

        trigger.setTargetState(0);
        trigger2.setTargetState(0);

        // s1.setVelocity(0.60);
        // s2.setVelocity(0.60);

        Thread::sleep(2000);
    }

    scheduler.stop();
    s1.setVelocity(0.5);
    s2.setVelocity(0.5);

    while (true)
    {
    };
}
