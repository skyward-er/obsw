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

#include "HIL.h"
#include "HILConfig.h"

class HILServo
{
public:
    /**
     * @brief constructor of the fake actuator used for the simulation.
     *
     * @param matlab reference of the MatlabTransceiver object that deals with
     * the simulator
     */
    HILServo() {}

    void enable()
    {
        // reset();
        isEnabled = true;
    }

    void disable()
    {
        // reset();
        isEnabled = false;
    }

    /**
     * @brief Initializes the fake actuator
     */
    bool init()
    {
        initialized = true;
        return true;
    }

    void selfTest() { return; }

    float getPosition()
    {
        miosix::Lock<FastMutex> l(mutex);
        return position;
    }

    /**
     * @brief sets the actuator data in the MatlabTransceiver object, then will
     * be sent to the simulator
     *
     * @param value opening in radiants
     */
    void setPosition(float value)
    {
        if (value < MinAlphaDegree)
            value = MinAlphaDegree;
        if (value > MaxAlphaDegree)
            value = MaxAlphaDegree;

        miosix::Lock<FastMutex> l(mutex);
        position = value;
        // TRACE("[HILServo] setting actuator\n");
        // actuatorData.print();
        // TRACE("[HILServo] didn't send abk opening\n");
        HIL::getInstance().send(value);
    }

    void sendToSimulator()
    {
        miosix::Lock<FastMutex> l(mutex);
        HIL::getInstance().send(position);
    }

    /*
     * converts the value that the real servo driver accepts to the value the
     * matlab simulator accepts
     */
    // float convertToDegree(float x) { return (x * 180 / PI); }

protected:
    miosix::FastMutex mutex;
    float position = 0;

    bool initialized = false;
    bool isEnabled   = true;
};
