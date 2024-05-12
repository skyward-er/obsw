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
using namespace Boardcore;

namespace Antennas
{

Leds::Leds(TaskScheduler* scheduler) : scheduler(scheduler)
{
    for (size_t i = 0; i < led_states.size(); i++)
    {
        led_states[i]  = LedState::OFF;
        led_mutexes[i] = make_unique<mutex>();
        led_cvs[i]     = make_unique<condition_variable>();
    }
}

bool Leds::start()
{
    size_t result;
    bool ok = true;

    // turn off all leds
    miosix::ledOff();

    // start the blinking thread for red led
    result = scheduler->addTask(
        std::bind(&Leds::ledThread, this, LedColor::RED, 100), 0,
        TaskScheduler::Policy::ONE_SHOT);
    ok &= result != 0;

    // start the blinking thread for yellow led
    result = scheduler->addTask(
        std::bind(&Leds::ledThread, this, LedColor::YELLOW, 50), 0,
        TaskScheduler::Policy::ONE_SHOT);
    ok &= result != 0;

    // start the blinking thread for orange led
    result = scheduler->addTask(
        std::bind(&Leds::ledThread, this, LedColor::ORANGE, 50), 0,
        TaskScheduler::Policy::ONE_SHOT);
    ok &= result != 0;

    // start the blinking thread for green led
    result = scheduler->addTask(
        std::bind(&Leds::ledThread, this, LedColor::GREEN, 50), 0,
        TaskScheduler::Policy::ONE_SHOT);
    ok &= result != 0;

    return ok;
}

void Leds::ledOn(LedColor color)
{
#ifdef _BOARD_STM32F767ZI_AUTOMATED_ANTENNAS
    switch (color)
    {
        case LedColor::RED:
            miosix::userLed4::high();
            break;
        case LedColor::YELLOW:
            miosix::led2On();
            break;
        case LedColor::ORANGE:
            miosix::led3On();
            break;
        case LedColor::GREEN:
            miosix::led1On();
            break;
    }
#endif
}

void Leds::ledOff(LedColor color)
{
#ifdef _BOARD_STM32F767ZI_AUTOMATED_ANTENNAS
    switch (color)
    {
        case LedColor::RED:
            miosix::userLed4::low();
            break;
        case LedColor::YELLOW:
            miosix::led2Off();
            break;
        case LedColor::ORANGE:
            miosix::led3Off();
            break;
        case LedColor::GREEN:
            miosix::led1Off();
            break;
    }
#endif
}

/**
 * @brief Thread function that actuate leds based on state
 * @param color led color to actuate
 * @param blinking_interval blinking time interval [ms]
 */
void Leds::ledThread(LedColor color, uint32_t blinking_interval)
{
    while (true)
    {
        unique_lock<mutex> lock(*mutexRef(color));
        cvRef(color)->wait(lock);
        switch (*ledRef(color))
        {
            case LedState::BLINKING:
            {
                do
                {
                    ledOn(color);
                    miosix::Thread::sleep(blinking_interval);
                    ledOff(color);
                    miosix::Thread::sleep(blinking_interval);
                } while (*ledRef(color) == LedState::BLINKING);
                break;
            }
            case LedState::ON:
            {
                ledOn(color);
            }
            case LedState::OFF:
            {
                ledOff(color);
            }
        }
    }
}

void Leds::setBlinking(LedColor color)
{
    lock_guard<mutex> lock(*mutexRef(color));
    *ledRef(color) = LedState::BLINKING;
    cvRef(color)->notify_one();
}

void Leds::setOn(LedColor color)
{
    lock_guard<mutex> lock(*mutexRef(color));
    *ledRef(color) = LedState::ON;
    cvRef(color)->notify_one();
}

void Leds::setOff(LedColor color)
{
    lock_guard<mutex> lock(*mutexRef(color));
    *ledRef(color) = LedState::OFF;
    cvRef(color)->notify_one();
}

void Leds::endlessBlink(LedColor color)
{
    lock_guard<mutex> lock(*mutexRef(color));
    *ledRef(color) = LedState::BLINKING;
    cvRef(color)->notify_one();
    miosix::Thread::wait();  // wait forever
}

}  // namespace Antennas
