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
#include <scheduler/TaskScheduler.h>

#include <array>
#include <mutex>
#include <utils/ModuleManager/ModuleManager.hpp>

constexpr uint32_t LED_BLINK_FAST_PERIOD_MS = 50;
constexpr uint32_t LED_BLINK_SLOW_PERIOD_MS = 300;

namespace Antennas
{

enum class LedColor : uint8_t
{
    RED = 0,
    YELLOW,
    ORANGE,
    BLUE,
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
     * @brief Start all the blinking LED thread
     */
    bool start();

    /**
     * @brief non-blocking action to set a led to blink in a loop
     */
    void setFastBlink(LedColor color);

    /**
     * @brief non-blocking action to set a led to blink in a loop
     */
    void setSlowBlink(LedColor color);

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
     * @note this method does not need any previous call to start()
     * and can be called as soon as the Leds object is created
     */
    void endlessBlink(LedColor color, uint32_t period);

private:
    enum class LedState : uint8_t
    {
        OFF = 0,
        ON,
        BLINK_SLOW,
        BLINK_FAST,
    };

    void ledOn(LedColor color);
    void ledOff(LedColor color);
    void ledToggle(LedColor color);

    /**
     * @brief Update routine called by the scheduler
     * @details This function is called by the scheduler to update the leds
     * state and blink them accordingly
     */
    void update();

    LedState* ledRef(LedColor color)
    {
        return &leds_state[static_cast<uint8_t>(color)];
    }

    // scheduler to run the update blink function
    Boardcore::TaskScheduler* scheduler;
    // toggles to allow led to blink
    std::array<bool, 5> led_toggles;
    // counter to keep track of slow blink
    std::array<uint32_t, 5> led_steps;
    // state of the leds
    std::array<LedState, 5> leds_state;
};

}  // namespace Antennas
