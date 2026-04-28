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

#include <Parafoil/Configs/ActuatorsConfig.h>
#include <Parafoil/StateMachines/WingController/WingControllerData.h>
#include <Parafoil/Wing/WingAlgorithmData.h>
#include <actuators/Servo/ServoWinch.h>
#include <algorithms/SchmittTrigger/SchmittTrigger.h>
#include <common/UnlimitedAngle.h>
#include <interfaces-impl/hwmapping.h>
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

namespace actuatorsConfig = Parafoil::Config::Actuators;

constexpr Degree SERVO1_ZERO_CALIBRATE = Degree(32.1f);
constexpr Degree SERV02_ZERO_CALIBRATE = Degree(351.2f);

GpioPin sckPin  = GpioPin(GPIOA_BASE, 5);
GpioPin misoPin = GpioPin(GPIOA_BASE, 6);
GpioPin mosiPin = GpioPin(GPIOA_BASE, 7);
GpioPin csPin   = GpioPin(GPIOA_BASE, 15);
GpioPin csPin2  = GpioPin(GPIOB_BASE, 7);

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

    // Initial setup for servo right
    ServoWinch servoRight(MIOSIX_PARAFOIL_SERVO_1_TIM,
                          TimerUtils::Channel::MIOSIX_PARAFOIL_SERVO_1_CHANNEL,
                          actuatorsConfig::RightServo::MIN_PULSE,
                          actuatorsConfig::RightServo::MAX_PULSE,
                          actuatorsConfig::RightServo::HERTZ);
    SchmittTrigger triggerServoRight(
        Radian(actuatorsConfig::RightServo::SCHMITT_THRESHOLD_LOW).value(),
        Radian(actuatorsConfig::RightServo::SCHMITT_THRESHOLD_HIGH).value());
    AS5047DSPIConfig configRight;
    configRight.dataType          = AS5047DDefs::DataSelect::DAECANG;
    configRight.daecEnabled       = AS5047DDefs::DAECStatus::DAEC_ON;
    configRight.rotationDirection = AS5047DDefs::ABIRotationDirection::NORMAL;
    AS5047DSPI as5047dServoRight(spiBus, csPin, configRight);
    Common::UnlimitedAngle angleSamplerRight;

    // Initial setup for servo left
    ServoWinch servoLeft(MIOSIX_PARAFOIL_SERVO_2_TIM,
                         TimerUtils::Channel::MIOSIX_PARAFOIL_SERVO_2_CHANNEL,
                         actuatorsConfig::LeftServo::MIN_PULSE,
                         actuatorsConfig::LeftServo::MAX_PULSE,
                         actuatorsConfig::LeftServo::HERTZ);
    SchmittTrigger triggerServoLeft(
        Radian(actuatorsConfig::LeftServo::SCHMITT_THRESHOLD_LOW).value(),
        Radian(actuatorsConfig::LeftServo::SCHMITT_THRESHOLD_HIGH).value());
    AS5047DSPIConfig configLeft;
    configLeft.dataType          = AS5047DDefs::DataSelect::DAECANG;
    configLeft.daecEnabled       = AS5047DDefs::DAECStatus::DAEC_ON;
    configLeft.rotationDirection = AS5047DDefs::ABIRotationDirection::NORMAL;
    AS5047DSPI as5047dServoLeft(spiBus, csPin2, configLeft);
    Common::UnlimitedAngle angleSamplerLeft;

    delayMs(10);
    if (!as5047dServoRight.init())
    {
        printf("Failed initialization servo right encoder\n");
        for (;;)
        {
        }
    }

    if (!as5047dServoLeft.init())
    {
        printf("Failed initialization servo left encoder\n");
        for (;;)
        {
        }
    }

    // Enable the timers
    servoRight.enable();
    servoLeft.enable();

    as5047dServoRight.sample();
    delayMs(1);
    as5047dServoLeft.sample();

    AS5047DData zeroAngleRight = as5047dServoRight.getLastSample();
    AS5047DData zeroAngleLeft  = as5047dServoLeft.getLastSample();

    triggerServoRight.begin();
    triggerServoLeft.begin();
    triggerServoRight.setTargetState(Radian(0).value());
    triggerServoLeft.setTargetState(Radian(0).value());
    angleSamplerRight.setInitialState(Radian(0));
    angleSamplerLeft.setInitialState(Radian(0));

    TaskScheduler scheduler;
    scheduler.addTask(
        [&]()
        {
            as5047dServoRight.sample();
            AS5047DData dataRight  = as5047dServoRight.getLastSample();
            auto angleReadingRight = angleSamplerRight.getUpdatedAngle(
                Radian(dataRight.angle - zeroAngleRight.angle));
            triggerServoRight.setCurrentState(angleReadingRight.value());
            triggerServoRight.update();
            SchmittTrigger::Activation outputRight =
                triggerServoRight.getOutput();

            switch (outputRight)
            {
                case SchmittTrigger::Activation::HIGH:
                {
                    servoRight.setVelocity(
                        actuatorsConfig::RightServo::HIGH_THRESHOLD_VELOCITY);
                    break;
                }
                case SchmittTrigger::Activation::LOW:
                {
                    servoRight.setVelocity(
                        actuatorsConfig::RightServo::LOW_THRESHOLD_VELOCITY);
                    break;
                }
                case SchmittTrigger::Activation::STOP:
                {
                    servoRight.setVelocity(
                        actuatorsConfig::RightServo::STOP_THRESHOLD_VELOCITY);
                    break;
                }
            }

            as5047dServoLeft.sample();
            AS5047DData dataLeft  = as5047dServoLeft.getLastSample();
            auto angleReadingLeft = angleSamplerLeft.getUpdatedAngle(
                Radian(dataLeft.angle - zeroAngleLeft.angle));

            triggerServoLeft.setCurrentState(angleReadingLeft.value());
            triggerServoLeft.update();

            SchmittTrigger::Activation outputLeft =
                triggerServoLeft.getOutput();
            switch (outputLeft)
            {
                case SchmittTrigger::Activation::HIGH:
                {
                    servoLeft.setVelocity(
                        actuatorsConfig::LeftServo::HIGH_THRESHOLD_VELOCITY);
                    break;
                }
                case SchmittTrigger::Activation::LOW:
                {
                    servoLeft.setVelocity(
                        actuatorsConfig::LeftServo::LOW_THRESHOLD_VELOCITY);
                    break;
                }
                case SchmittTrigger::Activation::STOP:
                {
                    servoLeft.setVelocity(
                        actuatorsConfig::LeftServo::STOP_THRESHOLD_VELOCITY);
                    break;
                }
            }

            auto dataLogger = Parafoil::WingControllerServoData{
                .timestamp          = TimestampTimer::getTimestamp(),
                .servo1AngleReading = Degree(angleReadingLeft).value(),
                .servo2AngleReading = Degree(angleReadingRight).value()};

            Logger::getInstance().log(dataLogger);
        },
        50_hz);

    std::cout << "Starting Logger" << std::endl;
    if (!Logger::getInstance().start())
    {
        std::cerr << "*** Failed to start Logger ***" << std::endl;

        if (!Logger::getInstance().testSDCard())
            std::cerr << "\tReason: SD card not present or not writable"
                      << std::endl;
        else
            std::cerr << "\tReason: Logger initialization error" << std::endl;
    }
    else
    {
        std::cout << "Logger Ok!\n"
                  << "\tLog number: "
                  << Logger::getInstance().getCurrentLogNumber() << std::endl;
    }

    scheduler.start();

    unsigned int currAngle  = 0;
    constexpr auto size     = sizeof anglesRight / sizeof(anglesRight[0]);
    constexpr auto sizeLeft = sizeof anglesLeft / sizeof(anglesLeft[0]);
    static_assert(size == sizeLeft,
                  "anglesRight size does not match anglesLeft size, check "
                  "test-parafoil-data.h");
    static_assert(size > 0,
                  "anglesRight and anglesLeft arrays must not be empty, check "
                  "test-parafoil-data.h");
    for (currAngle = 0; currAngle < size; currAngle++)
    {
        triggerServoRight.setTargetState(-anglesRight[currAngle]);
        triggerServoLeft.setTargetState(anglesLeft[currAngle]);
        Logger::getInstance().log(Parafoil::WingAlgorithmData{
            .timestamp   = TimestampTimer::getTimestamp(),
            .servo1Angle = Degree(anglesLeft[currAngle]).value(),
            .servo2Angle = Degree(-anglesRight[currAngle]).value()});

        Thread::sleep(20);
    }

    scheduler.stop();
    servoRight.setVelocity(0.5);
    servoLeft.setVelocity(0.5);

    while (true)
    {
    };
}
