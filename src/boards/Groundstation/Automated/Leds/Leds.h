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

#include <scheduler/TaskScheduler.h>

#include <array>
#include <condition_variable>
#include <map>
#include <mutex>
#include <utils/ModuleManager/ModuleManager.hpp>

namespace Antennas
{

enum class LedColor : uint8_t
{
    RED = 0,
    YELLOW,
    ORANGE,
    GREEN
};

/**
 * @brief Utility to handle blinking leds with non-blocking sleep
 * (useful for state machines states that need to blink leds without blocking)
 */
class Leds : public Boardcore::Module
{
public:
    explicit Leds(Boardcore::TaskScheduler* scheduler);

    /**
     * @brief Start the blinking LED thread
     */
    bool start();

    /**
     * @brief non-blocking action to set a led to blink in a loop
     */
    void setBlinking(LedColor color);

    /**
     * @brief blocking action to set on a led
     */
    void setOn(LedColor color);

    /**
     * @brief blocking action to set off a led
     */
    void setOff(LedColor color);

    /**
     * @brief blocking action to blink endlessly a led
     */
    void endlessBlink(LedColor color);

protected:
    enum class LedState : uint8_t
    {
        OFF = 0,
        ON,
        BLINKING
    };

    void ledOn(LedColor color);
    void ledOff(LedColor color);
    void ledThread(LedColor color, uint32_t ms_interval);

    LedState* ledRef(LedColor color)
    {
        return &led_states[static_cast<uint8_t>(color)];
    }

    std::mutex* mutexRef(LedColor color)
    {
        return led_mutexes[static_cast<uint8_t>(color)].get();
    }

    std::condition_variable* cvRef(LedColor color)
    {
        return led_cvs[static_cast<uint8_t>(color)].get();
    }

    Boardcore::TaskScheduler* scheduler;
    std::array<LedState, 4> led_states;
    std::array<unique_ptr<std::mutex>, 4> led_mutexes;
    std::array<unique_ptr<std::condition_variable>, 4> led_cvs;
};

}  // namespace Antennas
