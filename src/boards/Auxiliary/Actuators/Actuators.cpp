/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include "Actuators.h"

#include <Auxiliary/BoardScheduler.h>
#include <common/LedConfig.h>
#include <interfaces-impl/bsp_impl.h>

using namespace miosix;
using namespace Boardcore;
using namespace Common;

namespace Auxiliary
{

Actuators::Actuators() : led(leds::led1::getPin()) {}

void Actuators::camOn() { interfaces::camMosfet::high(); }

void Actuators::camOff() { interfaces::camMosfet::low(); }

void Actuators::ledArmed()
{
    TaskScheduler &scheduler = BoardScheduler::getInstance().getScheduler();
    scheduler.removeTask(ledTaskId);
    ledTaskId = scheduler.addTask([&]() { toggleLed(); }, LED_ARMED_PERIOD);
}

void Actuators::ledDisarmed()
{
    BoardScheduler::getInstance().getScheduler().removeTask(ledTaskId);
    led.high();
    ledState = true;
}

void Actuators::ledError()
{
    TaskScheduler &scheduler = BoardScheduler::getInstance().getScheduler();
    scheduler.removeTask(ledTaskId);
    ledTaskId = scheduler.addTask([&]() { toggleLed(); }, LED_ERROR_PERIOD);
}

void Actuators::ledOff()
{
    BoardScheduler::getInstance().getScheduler().removeTask(ledTaskId);
    ledTaskId = 0;
    ledState  = false;
    led.low();
}

void Actuators::toggleLed()
{
    if (ledState)
        led.high();
    else
        led.low();

    ledState = !ledState;
}

}  // namespace Auxiliary
