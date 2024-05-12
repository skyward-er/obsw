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

#include <mutex>

using namespace std;
using namespace miosix;
using namespace Boardcore;

namespace Antennas
{

Leds::Leds(TaskScheduler* scheduler) : scheduler(scheduler)
{
    *led_ref(LedColor::RED)    = LedState::OFF;
    *led_ref(LedColor::YELLOW) = LedState::OFF;
    *led_ref(LedColor::ORANGE) = LedState::OFF;
    *led_ref(LedColor::GREEN)  = LedState::OFF;
}

bool Leds::start()
{
    size_t result;
    bool ok = true;

    // turn off all leds
    ledOff();

    // start the blinking thread for red led
    result = scheduler->addTask(
        std::bind(&Leds::led_thread, this, LedColor::RED, 100), 0,
        TaskScheduler::Policy::ONE_SHOT);
    ok &= result != 0;

    // start the blinking thread for yellow led
    result = scheduler->addTask(
        std::bind(&Leds::led_thread, this, LedColor::YELLOW, 50), 0,
        TaskScheduler::Policy::ONE_SHOT);
    ok &= result != 0;

    // start the blinking thread for orange led
    result = scheduler->addTask(
        std::bind(&Leds::led_thread, this, LedColor::ORANGE, 50), 0,
        TaskScheduler::Policy::ONE_SHOT);
    ok &= result != 0;

    // start the blinking thread for green led
    result = scheduler->addTask(
        std::bind(&Leds::led_thread, this, LedColor::GREEN, 50), 0,
        TaskScheduler::Policy::ONE_SHOT);
    ok &= result != 0;

    return ok;
}

void Leds::led_on(LedColor color)
{
#ifdef _BOARD_STM32F767ZI_AUTOMATED_ANTENNAS
    switch (color)
    {
        case LedColor::RED:
            userLed4::high();
            break;
        case LedColor::YELLOW:
            led2On();
            break;
        case LedColor::ORANGE:
            led3On();
            break;
        case LedColor::GREEN:
            led1On();
            break;
    }
#endif
}

void Leds::led_off(LedColor color)
{
#ifdef _BOARD_STM32F767ZI_AUTOMATED_ANTENNAS
    switch (color)
    {
        case LedColor::RED:
            userLed4::low();
            break;
        case LedColor::YELLOW:
            led2Off();
            break;
        case LedColor::ORANGE:
            led3Off();
            break;
        case LedColor::GREEN:
            led1Off();
            break;
    }
#endif
}

/**
 * @brief Thread function that actuate leds based on state
 * @param color led color to actuate
 * @param blinking_interval blinking time interval [ms]
 */
void Leds::led_thread(LedColor color, uint32_t blinking_interval)
{
    while (true)
    {
        unique_lock<mutex> lock(*mutex_ref(color));
        cv_ref(color)->wait(lock);
        switch (*led_ref(color))
        {
            case LedState::BLINKING:
            {
                do
                {
                    led_on(color);
                    Thread::sleep(blinking_interval);
                    led_off(color);
                    Thread::sleep(blinking_interval);
                } while (*led_ref(color) == LedState::BLINKING);
                break;
            }
            case LedState::ON:
            {
                led_on(color);
            }
            case LedState::OFF:
            {
                led_off(color);
            }
        }
    }
}

void Leds::set_blinking(LedColor color)
{
    lock_guard<mutex> lock(*mutex_ref(color));
    *led_ref(color) = LedState::BLINKING;
    cv_ref(color)->notify_one();
}

void Leds::set_on(LedColor color)
{
    lock_guard<mutex> lock(*mutex_ref(color));
    *led_ref(color) = LedState::ON;
    cv_ref(color)->notify_one();
}

void Leds::set_off(LedColor color)
{
    lock_guard<mutex> lock(*mutex_ref(color));
    *led_ref(color) = LedState::OFF;
    cv_ref(color)->notify_one();
}

void Leds::endless_blink(LedColor color)
{
    lock_guard<mutex> lock(*mutex_ref(color));
    *led_ref(color) = LedState::BLINKING;
    cv_ref(color)->notify_one();
    Thread::wait();  // wait forever
}

}  // namespace Antennas
