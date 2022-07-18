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

#include "HILConfig.h"
#include "common/Algorithm.h"
#include "common/ServoInterface.h"
#include "miosix.h"
#include "sensors/Sensor.h"

/**
 * @brief Example of how a control algorithm should be created.
 *
 * The control algorithm takes data from the Sensors, elaborates it and sends
 * the result to the actuator.
 */
template <typename KD>
class MockAirbrakeAlgorithm : public Algorithm
{
public:
    /**
     * @brief constructor that takes the state of the rocket and the actuator
     * @param kalman HILSensor that has as data structure the first templated
     * parameter
     * @param servo the actuator that will communicate with the Simulator
     */
    MockAirbrakeAlgorithm(Boardcore::Sensor<KD>* kalman, ServoInterface* servo)
    {
        /* [TODO]
         * Check if the sensors have the data we need
         */
        this->kalman         = kalman;
        this->servo          = servo;
        lastSample.timestamp = 0;
    }

    bool init() override { return true; }

protected:
    /**
     * @brief Example of computation on the data sent by the Simulator
     */
    void step() override
    {
        KD state;
        {
            // [REVIEW] PauseKernelLock kLock;
            state = kalman->getLastSample();
        }

        if (lastSample.timestamp < state.timestamp)
        {
            lastSample.timestamp = state.timestamp;

            /*
             * In here you can put the control algorithm
             */

            actuatorData = MaxAlphaDegree / 2;  // it's more or less 25 degrees
        }

        servo->set(actuatorData);
    }

private:
    ActuatorData actuatorData;
    Boardcore::TimestampData
        lastSample;                /**< keeps track of the last timestamp */
    Boardcore::Sensor<KD>* kalman; /**< reference to the kalman object */
    ServoInterface* servo;         /**< reference to the actuator object */
};
