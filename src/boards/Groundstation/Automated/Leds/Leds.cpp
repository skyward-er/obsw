/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Federico Lolli
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

#include "Leds.h"

using namespace Boardcore;

namespace Antennas
{

Leds::Leds(TaskScheduler* scheduler) : scheduler(scheduler)
{
    leds_state.fill(LedState::OFF);
    led_toggles.fill(false);
    led_steps.fill(0);
}

bool Leds::start()
{
    size_t result;
    bool ok = true;

    // turn off all leds
    miosix::ledOff();

    result = scheduler->addTask(std::bind(&Leds::update, this),
                                LED_BLINK_FAST_PERIOD_MS,
                                TaskScheduler::Policy::RECOVER);
    ok &= result;

    return ok;
}

void Leds::update()
{
    LedState state;
    LedColor color;

    for (uint8_t i = 0; i < leds_state.size(); i++)
    {
        state = leds_state[i];
        color = static_cast<LedColor>(i);

        switch (state)
        {
            case LedState::BLINK_SLOW:
                if (led_steps[i]++ >= 1)
                {
                    ledToggle(color);
                    led_steps[i] = 0;
                }
                break;
            case LedState::BLINK_FAST:
            {
                ledToggle(color);
                break;
            }
            case LedState::ON:
            {
                ledOn(color);
                break;
            }
            case LedState::OFF:
            {
                ledOff(color);
                break;
            }
        }
    }
}

void Leds::setFastBlink(LedColor color)
{
    *ledRef(color) = LedState::BLINK_FAST;
}

void Leds::setSlowBlink(LedColor color)
{
    *ledRef(color) = LedState::BLINK_SLOW;
}

void Leds::setOn(LedColor color) { *ledRef(color) = LedState::ON; }

void Leds::setOff(LedColor color) { *ledRef(color) = LedState::OFF; }

void Leds::endlessBlink(LedColor color, uint32_t period)
{
    while (true)
    {
        ledToggle(color);
        sleep(period);
    }
}

void Leds::ledToggle(LedColor color)
{
    uint8_t i = static_cast<uint8_t>(color);
    led_toggles[i] ? ledOn(color) : ledOff(color);
    led_toggles[i] = !led_toggles[i];
}

void Leds::ledOn(LedColor color)
{
    switch (color)
    {
        case LedColor::RED:
            miosix::led3On();
            miosix::commBox::ledTimR2::high();
            break;
        case LedColor::YELLOW:
            miosix::led2On();
            miosix::commBox::ledTimY1::high();

            break;
        case LedColor::ORANGE:
            miosix::led4On();
            break;
        case LedColor::GREEN:
            miosix::led1On();
            break;
        case LedColor::BLUE:
            miosix::commBox::ledTimB3::high();
            break;
    }
}

void Leds::ledOff(LedColor color)
{
    switch (color)
    {
        case LedColor::RED:
            miosix::led3Off();
            miosix::commBox::ledTimR2::low();
            break;
        case LedColor::YELLOW:
            miosix::led2Off();
            miosix::commBox::ledTimY1::low();
            break;
        case LedColor::ORANGE:
            miosix::led4Off();
            break;
        case LedColor::GREEN:
            miosix::led1Off();
            break;
        case LedColor::BLUE:
            miosix::commBox::ledTimB3::low();
            break;
    }
}

}  // namespace Antennas
