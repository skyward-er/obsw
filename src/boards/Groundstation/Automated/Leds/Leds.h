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

#include <array>
#include <condition_variable>
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

class LedThread : public Boardcore::ActiveObject
{
public:
    explicit LedThread(LedColor color);

    /**
     * @brief non-blocking action to set a led to blink in a loop
     */
    void setBlinking(uint32_t ms_interval);

    /**
     * @brief non-blocking action to set on the led
     */
    void setOn();

    /**
     * @brief non-blocking action to set off the led
     */
    void setOff();

private:
    enum class LedState : uint8_t
    {
        OFF = 0,
        ON,
        BLINKING
    };

    void ledOn(LedColor color);
    void ledOff(LedColor color);
    void run() override;

    LedColor color;
    LedState state;
    std::mutex mutex;
    std::condition_variable cv;
    uint32_t blinking_interval = 100;  // [ms]
};

/**
 * @brief Utility to handle blinking leds with non-blocking sleep
 * (useful for state machines states that need to blink leds without blocking)
 */
class Leds : public Boardcore::Module
{
public:
    explicit Leds();

    /**
     * @brief Start all the blinking LED thread
     */
    bool start();

    /**
     * @brief non-blocking action to set a led to blink in a loop
     */
    void setBlinking(LedColor color, uint32_t ms_interval);

    /**
     * @brief non-blocking action to set on a led
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

private:
    LedThread* ledRef(LedColor color)
    {
        return leds[static_cast<uint8_t>(color)].get();
    }

    std::array<std::unique_ptr<LedThread>, 4> leds;
};

}  // namespace Antennas
