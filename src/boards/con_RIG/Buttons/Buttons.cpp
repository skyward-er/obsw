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
#include <con_RIG/Buses.h>
#include <con_RIG/Configs/ButtonsConfig.h>

using namespace std;
using namespace miosix;
using namespace Boardcore;
using namespace con_RIG::ButtonsConfig;

namespace con_RIG
{

Buttons::Buttons()
{
    resetState();
    state.armed = false;

    ModuleManager& modules = ModuleManager::getInstance();
    modules.get<BoardScheduler>()->getScheduler().addTask(
        [&]() { periodicStatusCheck(); }, BUTTON_SAMPLE_PERIOD,
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
    state.fillin_valve                  = false;
    state.venting_valve                 = false;
    state.release_filling_line_pressure = false;
    state.detach_quick_connector        = false;
    state.startup_tars                  = false;
}

void Buttons::periodicStatusCheck()
{

    // TODO: This should be in bsp
    using GpioIgnitionBtn        = Gpio<GPIOB_BASE, 4>;
    using GpioFillinValveBtn     = Gpio<GPIOE_BASE, 6>;
    using GpioVentingValveBtn    = Gpio<GPIOE_BASE, 4>;
    using GpioReleasePressureBtn = Gpio<GPIOG_BASE, 9>;
    using GpioQuickConnectorBtn  = Gpio<GPIOD_BASE, 7>;
    using GpioStartTarsBtn       = Gpio<GPIOD_BASE, 5>;
    using GpioArmedSwitch        = Gpio<GPIOE_BASE, 2>;

    using armed_led = Gpio<GPIOC_BASE, 13>;
    armed_led::high();

    state.armed = GpioArmedSwitch::getPin().value();

    if (!GpioIgnitionBtn::getPin().value() && state.armed)
    {
        state.ignition = true;
    }
    if (GpioFillinValveBtn::getPin().value())
    {
        state.fillin_valve = true;
    }
    if (GpioVentingValveBtn::getPin().value())
    {
        state.venting_valve = true;
    }
    if (GpioReleasePressureBtn::getPin().value())
    {
        state.release_filling_line_pressure = true;
    }
    if (GpioQuickConnectorBtn::getPin().value())
    {
        state.release_filling_line_pressure = true;
    }
    if (GpioStartTarsBtn::getPin().value())
    {
        state.startup_tars = true;
    }
}

// 1 if rocket is armed, 0 if disarmed
void Buttons::setRemoteArmState(int armed)
{
    // TODO: This should be in bsp
    // TODO: fix gpio values
    using armed_led = Gpio<GPIOC_BASE, 13>;
    if (armed == 1)
    {
        armed_led::high();
    }
    else
    {
        armed_led::low();
    }
}

}  // namespace con_RIG
