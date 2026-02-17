/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Riccardo Sironi
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

#include <miosix.h>

#include <memory>

#include "ValveInterface.h"
#include "actuators/Servo/Servo.h"

namespace RIGv2
{
class ValveServo : public ValveInterface
{
public:
    /**
     * @brief ValveSolenoid Constructor
     * @param pin Solenoid valve control pin
     */
    ValveServo(std::unique_ptr<Boardcore::Servo> servo)
        : servo(std::move(servo))
    {
    }
    ~ValveServo() {};

    // Move-only
    ValveServo(ValveServo&&)                 = default;
    ValveServo& operator=(ValveServo&&)      = default;
    ValveServo(const ValveServo&)            = delete;
    ValveServo& operator=(const ValveServo&) = delete;

    /**
     * @brief Sets the state of the solenoid valve (open/closed).
     * @param position position values greater than 0.5f are treated as high.
     */
    void setPosition(float position, bool limited = false)
    {
        servo->setPosition(position, limited);
    };

    /**
     * @brief Returns the state of the solenoid valve (open/closed).
     * @returns True if open
     *
     * False if closed
     */
    float getPosition() { return servo->getPosition(); };

    void enable() override { servo->enable(); }

    ValveType getType() const override { return ValveType::TIMED; }

private:
    std::unique_ptr<Boardcore::Servo> servo;
};
}  // namespace RIGv2
