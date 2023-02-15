/* Copyright (c) 2020 Skyward Experimental Rocketry
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

#include <Main/Actuators/Actuators.h>
#include <algorithms/Algorithm.h>

#include "HILConfig.h"
#include "miosix.h"
#include "sensors/Sensor.h"

/**
 * @brief Example of how a control algorithm should be created.
 *
 * The control algorithm takes data from the Sensors, elaborates it and sends
 * the result to the actuator.
 */
class MockAirbrakeAlgorithm : public Boardcore::Algorithm
{
public:
    /**
     * @brief constructor that takes the state of the rocket and the actuator
     * @param kalman HILSensor that has as data structure the first templated
     * parameter
     * @param servo the actuator that will communicate with the Simulator
     */
    MockAirbrakeAlgorithm(
        std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition)
        : getCurrentPosition(getCurrentPosition)
    {
        lastSample.timestamp = 0;
    }

    bool init() override { return true; }

protected:
    /**
     * @brief Example of computation on the data sent by the Simulator
     */
    void step() override
    {
        Boardcore::TimedTrajectoryPoint state;
        {
            // [REVIEW] PauseKernelLock kLock;
            state = getCurrentPosition();
        }

        if (lastSample.timestamp < state.timestamp)
        {
            lastSample.timestamp = state.timestamp;

            /*
             * In here you can put the control algorithm
             */

            // setting the opening to 50%
            Main::Actuators::getInstance().setServo(
                ServosList::AIR_BRAKES_SERVO, 0.5);
            // Main::Actuators::getInstance().sendToSimulator();
        }
    }

private:
    Boardcore::TimestampData
        lastSample; /**< keeps track of the last timestamp */
    std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition;
    std::function<void(float)> setActuator;
};
