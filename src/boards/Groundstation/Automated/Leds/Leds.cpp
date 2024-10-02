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

using namespace Boardcore;

namespace Antennas
{

LedThread::LedThread(LedColor color)
    : ActiveObject(miosix::STACK_DEFAULT_FOR_PTHREAD, 0), color(color),
      state(LedState::OFF)
{
}

void LedThread::ledOn(LedColor color)
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
#else
    switch (color)
    {
        case LedColor::RED:
            miosix::ledOn();
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

void LedThread::ledOff(LedColor color)
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
#else
    switch (color)
    {
        case LedColor::RED:
            miosix::ledOff();
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

void LedThread::run()
{
    LedState old = state;
    while (true)
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&] { return state != old; });
        old = state;
        switch (state)
        {
            case LedState::BLINKING:
            {
                do
                {
                    ledOn(color);
                    miosix::Thread::sleep(blinking_interval);
                    ledOff(color);
                    miosix::Thread::sleep(blinking_interval);
                } while (state == LedState::BLINKING);
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

void LedThread::setBlinking(uint32_t ms_interval)
{
    {
        std::lock_guard<std::mutex> lock(mutex);
        state = LedState::BLINKING;
    }
    cv.notify_one();
}

void LedThread::setOn()
{
    {
        std::lock_guard<std::mutex> lock(mutex);
        state = LedState::ON;
    }
    cv.notify_one();
}

void LedThread::setOff()
{
    {
        std::lock_guard<std::mutex> lock(mutex);
        state = LedState::OFF;
    }
    cv.notify_one();
}

Leds::Leds()
{
    for (size_t i = 0; i < leds.size(); i++)
    {
        leds[i] = std::make_unique<LedThread>(static_cast<LedColor>(i));
    }
}

bool Leds::start()
{
    bool ok = true;

    // turn off all leds
    miosix::ledOff();

    for (size_t i = 0; i < leds.size(); i++)
    {
        ok &= leds[i]->start();
    }

    return ok;
}

void Leds::setBlinking(LedColor color, uint32_t ms_interval)
{
    ledRef(color)->setBlinking(ms_interval);
}

void Leds::setOn(LedColor color) { ledRef(color)->setOn(); }

void Leds::setOff(LedColor color) { ledRef(color)->setOff(); }

void Leds::endlessBlink(LedColor color)
{
    ledRef(color)->setBlinking(100);
    miosix::Thread::wait();  // wait forever
}

}  // namespace Antennas
