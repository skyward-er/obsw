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
#include <con_RIG/Radio/Radio.h>
#include <interfaces-impl/hwmapping.h>

using namespace std;
using namespace miosix;
using namespace Boardcore;
using namespace con_RIG;

Buttons::Buttons(TaskScheduler& scheduler) : scheduler(scheduler)
{
    resetState();
    state.arm_switch = false;
}

bool Buttons::start()
{
    return scheduler.addTask([this]() { periodicStatusCheck(); },
                             Config::Buttons::BUTTON_SAMPLE_PERIOD) != 0;
}

mavlink_conrig_state_tc_t Buttons::getState() { return state; }

void Buttons::resetState()
{
    state.ignition_btn         = false;
    state.filling_valve_btn    = false;
    state.venting_valve_btn    = false;
    state.release_pressure_btn = false;
    state.quick_connector_btn  = false;
    state.start_tars_btn       = false;
}

void Buttons::periodicStatusCheck()
{
    state.arm_switch = btns::arm::value();

    if (!btns::ignition::value() && state.arm_switch)
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard              = 0;
            state.ignition_btn = true;
        }
        else
        {
            guard++;
        }
    }
    else if (btns::filling::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard                   = 0;
            state.filling_valve_btn = true;
        }
        else
        {
            guard++;
        }
    }
    else if (btns::venting::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard                   = 0;
            state.venting_valve_btn = true;
        }
        else
        {
            guard++;
        }
    }
    else if (btns::release::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard                      = 0;
            state.release_pressure_btn = true;
        }
        else
        {
            guard++;
        }
    }
    else if (btns::detach::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard                     = 0;
            state.quick_connector_btn = true;
        }
        else
        {
            guard++;
        }
    }
    else if (btns::tars::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard                = 0;
            state.start_tars_btn = true;
        }
        else
        {
            guard++;
        }
    }
    else
    {
        // Reset all the states and guard
        guard = 0;
        resetState();
    }

    // Set the internal button state in Radio module
    ModuleManager::getInstance().get<Radio>()->setInternalState(state);

    // printf("%d %d %d %d %d %d %d\n", state.ignition, state.filling_valve,
    //        state.venting_valve, state.release_filling_line_pressure,
    //        state.detach_quick_connector, state.startup_tars, state.armed);
}

void Buttons::enableIgnition() { ui::armedLed::high(); }

void Buttons::disableIgnition() { ui::armedLed::low(); }
