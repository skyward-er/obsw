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

#pragma once

#include <ActiveObject.h>
#include <miosix.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Antennas
{

/**
 * @brief Utility to handle blinking leds with non-blocking sleep
 * (useful for state machines states that need to blink leds without blocking)
 */
class Leds : public Boardcore::Module, protected Boardcore::ActiveObject
{
public:
    Leds() {}

    ~Leds() { ActiveObject::stop(); }

    /**
     * @brief Start the blinking LED thread
     */
    bool start();

    /**
     * @brief start blinking red led
     */
    void start_blinking_red() { blinking_red = true; }

    /**
     * @brief stop blinking red led
     */
    void stop_blinking_red() { blinking_red = false; }

    /**
     * @brief turn on green led
     */
    void turn_on_green() { led1On(); }

    /**
     * @brief start blinking yellow led
     */
    void start_blinking_yellow() { yellow_blink = true; }

    /**
     * @brief stop blinking yellow led
     */
    void stop_blinking_yellow()
    {
        yellow_blink = false;
        Thread::sleep(101);  // to avoid data races with led2On/Off
    }

    /**
     * @brief turn on yellow led
     */
    void turn_on_yellow() { led2On(); }

    /**
     * @brief turn off yellow led
     */
    void turn_off_yellow() { led2Off(); }

    /**
     * @brief start blinking orange led
     */
    void start_blinking_orange() { orange_blinking = true; }

    /**
     * @brief stop blinking orange led
     */
    void stop_blinking_orange()
    {
        orange_blinking = false;
        Thread::sleep(101);  // to avoid data races with led3On/Off
    }

    /**
     * @brief turn on orange led
     */
    void turn_on_orange() { led3On(); }

    /**
     * @brief turn off orange led
     */
    void turn_off_orange() { led3Off(); }

    /**
     * @brief Blink the red led in a loop, blocking the thread.
     */
    static void errorLoop();

protected:
    void run() override;

private:
    std::atomic_bool blinking_red{false};
    std::atomic_bool yellow_blink{false};
    std::atomic_bool orange_blinking{false};
};

}  // namespace Antennas
