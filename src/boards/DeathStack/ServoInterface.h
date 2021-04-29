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

namespace DeathStackBoard
{

/**
 * @brief Class for interfacing with 180° servo motors, works in degrees.
 *
 * This class provides all the methods for enabling/disabling servo
 * communication as well as methods for testing it (like reset, setMaxPosition
 * and selftest).
 * The function set(float) is used to send data to the servo and calls, in
 * sequence, the preprocessPosition() and the setPosition() functions. This
 * function must not be overridden and all the logic of preprocessing and
 * sending position to the actual servo must be implemented via those two
 * methods.
 * */
class ServoInterface
{
public:
    /**
     * @brief Class constructor.
     *
     * @param minPosition The minimum position possible for the Servo
     * @param maxPosition The maximum position possible for the Servo
     * */
    ServoInterface(float minPosition, float maxPosition)
        : MIN_POS(minPosition), MAX_POS(maxPosition), RESET_POS(minPosition)
    {
    }

    /**
     * @brief Class constructor.
     *
     * @param minPosition The minimum position possible for the Servo
     * @param maxPosition The maximum position possible for the Servo
     * @param resetPosition The reset position for the Servo
     * */
    ServoInterface(float minPosition, float maxPosition, float resetPosition)
        : MIN_POS(minPosition), MAX_POS(maxPosition), RESET_POS(resetPosition)
    {
    }
 
    virtual ~ServoInterface()
    {

    }
    
    /**
     * @brief Enables the communication with the servo and sets it to its reset
     * position.
     * */
    virtual void enable() = 0;

    /**
     * @brief Disables the communication with the servo.
     * */
    virtual void disable() = 0;

    /**
     * @brief Sends the given input to the Servo. Before sending data the input
     * is preprocessed in order to have a physical consistent position to send
     * to the servo. Do not override this method, but use @see{setPosition} and
     * @see{preprocessPosition}.
     *
     * @param angle The input to be sent to the Servo
     * */
    void set(float angle) { setPosition(preprocessPosition(angle)); }

    /**
     * @brief Sends the Servo the highest input possible
     * */
    void setMaxPosition() { setPosition(MAX_POS); }

    /**
     * @brief Sends the Servo the lowest input possible
     * */
    void setMinPosition() { setPosition(MIN_POS); }

    /**
     * @brief Sets servo to its reset position
     * */
    void reset() { setPosition(RESET_POS); }

    /**
     * @return current actuator position (in degrees)
     */
    float getCurrentPosition()
    {
        return currentPosition;
    }

    virtual void selfTest() = 0;

    const float MIN_POS   = 0;
    const float MAX_POS   = 0;
    const float RESET_POS = 0;

protected:
    /**
     * @brief Sends the data to the servo
     *
     * @param angle Data to be sent to servo
     * */
    virtual void setPosition(float angle) = 0;

    /**
     * @brief Applies any transformation needed to the data before actually
     * sending it to the servo
     *
     * @param angle Non normalized input position
     *
     * @returns Normalized input position
     * */
    virtual float preprocessPosition(float angle)
    {
        if (angle > MAX_POS)
        {
            angle = MAX_POS;
        }
        else if (angle < MIN_POS)
        {
            angle = MIN_POS;
        }

        return angle;
    };

    /**
     * @brief Actuator's current position.
     */
    float currentPosition = 0;
};

}  // namespace DeathStackBoard
