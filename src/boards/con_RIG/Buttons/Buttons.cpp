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
    isArmed  = false;
    wasArmed = false;
    resetState();

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
    wasArmed                            = isArmed;
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
    // TODO: fix gpio values
    using GpioIgnitionBtn        = Gpio<GPIOA_BASE, 0>;
    using GpioFillinValveBtn     = Gpio<GPIOA_BASE, 1>;
    using GpioVentingValveBtn    = Gpio<GPIOA_BASE, 2>;
    using GpioReleasePressureBtn = Gpio<GPIOA_BASE, 3>;
    using GpioQuickConnectorBtn  = Gpio<GPIOA_BASE, 4>;
    using GpioStartTarsBtn       = Gpio<GPIOA_BASE, 5>;

    // Note: The button is assumed to be pressed if the pin value is low
    // (pulldown)
    if (!GpioIgnitionBtn::getPin().value())
    {
        state.ignition = true;
    }
    if (!GpioFillinValveBtn::getPin().value())
    {
        state.fillin_valve = true;
    }
    if (!GpioVentingValveBtn::getPin().value())
    {
        state.venting_valve = true;
    }
    if (!GpioReleasePressureBtn::getPin().value())
    {
        state.release_filling_line_pressure = true;
    }
    if (!GpioQuickConnectorBtn::getPin().value())
    {
        state.release_filling_line_pressure = true;
    }
    if (!GpioStartTarsBtn::getPin().value())
    {
        state.startup_tars = true;
    }

    using GpioArmedSwitch = Gpio<GPIOA_BASE, 6>;
    isArmed               = !GpioArmedSwitch::getPin().value();
}

// 0 if nothing changes, 1 if should arm, -1 if should disarm
int Buttons::shouldArm()
{
    if (isArmed && !wasArmed)
    {
        return 1;
    }
    if (wasArmed && !isArmed)
    {
        return -1;
    }
    return 0;
}

// 1 if rocket is armed, 0 if disarmed
void Buttons::setRemoteArmState(int armed)
{
    // TODO: This should be in bsp
    // TODO: fix gpio values
    using armed_led = Gpio<GPIOC_BASE, 9>;
    if (armed)
    {
        armed_led::high();
    }
    else
    {
        armed_led::low();
    }
}

}  // namespace con_RIG
