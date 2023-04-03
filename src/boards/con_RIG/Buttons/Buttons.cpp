/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Giacomo Caironi
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

#include "Buttons.h"

#include <con_RIG/BoardScheduler.h>
#include <con_RIG/Configs/ButtonsConfig.h>

using namespace std;
using namespace miosix;
using namespace Boardcore;
using namespace con_RIG::Config::Buttons;

namespace con_RIG
{

Buttons::Buttons(TaskScheduler* sched) : scheduler(sched)
{
    resetState();
    state.armed = false;
    remoteArm   = 0;
}

bool Buttons::start()
{
    return scheduler->addTask([&]() { periodicStatusCheck(); },
                              CHECK_BUTTON_STATE_TASK_ID);
}

Buttons::~Buttons()
{
    // Delete all the buttons
}

ButtonsState Buttons::getState() { return state; }

void Buttons::resetState()
{
    state.ignition                      = false;
    state.filling_valve                 = false;
    state.venting_valve                 = false;
    state.release_filling_line_pressure = false;
    state.detach_quick_connector        = false;
    state.startup_tars                  = false;
}

void Buttons::periodicStatusCheck()
{
    // TODO: This should be in bsp
    using GpioIgnitionBtn        = Gpio<GPIOB_BASE, 4>;
    using GpioFillingValveBtn    = Gpio<GPIOE_BASE, 6>;
    using GpioVentingValveBtn    = Gpio<GPIOE_BASE, 4>;
    using GpioReleasePressureBtn = Gpio<GPIOG_BASE, 9>;
    using GpioQuickConnectorBtn  = Gpio<GPIOD_BASE, 7>;
    using GpioStartTarsBtn       = Gpio<GPIOD_BASE, 5>;
    using GpioArmedSwitch        = Gpio<GPIOE_BASE, 2>;

    state.armed = GpioArmedSwitch::getPin().value();

    if (!GpioIgnitionBtn::getPin().value() && state.armed)
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard          = 0;
            state.ignition = true;
        }
        else
        {
            guard++;
        }
    }
    else if (GpioFillingValveBtn::getPin().value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard               = 0;
            state.filling_valve = true;
        }
        else
        {
            guard++;
        }
    }
    else if (GpioVentingValveBtn::getPin().value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard               = 0;
            state.venting_valve = true;
        }
        else
        {
            guard++;
        }
    }
    else if (GpioReleasePressureBtn::getPin().value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard                               = 0;
            state.release_filling_line_pressure = true;
        }
        else
        {
            guard++;
        }
    }
    else if (GpioQuickConnectorBtn::getPin().value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard                        = 0;
            state.detach_quick_connector = true;
        }
        else
        {
            guard++;
        }
    }
    else if (GpioStartTarsBtn::getPin().value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard              = 0;
            state.startup_tars = true;
        }
        else
        {
            guard++;
        }
    }
    else
    {
        // Reset the guard
        guard = 0;
    }
}

// 1 if rocket is armed, 0 if disarmed
void Buttons::setRemoteArmState(int armed)
{
    using armed_led = Gpio<GPIOC_BASE, 13>;

    if (armed)
    {
        remoteArm = true;
        armed_led::high();
    }
    else
    {
        remoteArm = false;
        armed_led::low();
    }
}

}  // namespace con_RIG
