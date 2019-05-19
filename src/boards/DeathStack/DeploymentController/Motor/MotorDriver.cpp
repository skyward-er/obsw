/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron
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
#include <Common.h>
#include <interfaces-impl/hwmapping.h>

#include <events/EventBroker.h>
#include "DeathStack/Events.h"
#include "MotorDriver.h"

using namespace miosix;

namespace DeathStackBoard
{

MotorDriver::MotorDriver() { status.motor_active = 0; }

MotorDriver::~MotorDriver() { stop(); }

bool MotorDriver::start(MotorDirection direction)
{
    // Ensure the pins are configured in output mode. (The rogallo controller
    // may set these pins to "alternate mode" to use PWM to drive the servos)
    nosecone::motP1::mode(Mode::OUTPUT);
    nosecone::motP2::mode(Mode::OUTPUT);
    nosecone::motP1::low();
    nosecone::motP2::low();

    if (direction == MotorDirection::NORMAL)
    {
        nosecone::motP1::high();
        nosecone::motP2::low();
    }
    else
    {
        nosecone::motP1::low();
        nosecone::motP2::high();
    }

    /* Activate PWM on both half bridges */
    nosecone::motEn::high();

    /* Update status */
    status.motor_active         = true;
    status.motor_last_direction = direction;

    return true;
}

void MotorDriver::stop()
{
    nosecone::motP1::low();
    nosecone::motP2::low();

    Thread::sleep(MOTOR_DISABLE_DELAY_MS);  // Wait a short delay

    nosecone::motEn::low();

    status.motor_active = false;
}

}  // namespace DeathStackBoard