/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Ettore Pane
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

#include <ConRIGv2/BoardScheduler.h>
#include <ConRIGv2/Configs/ButtonsConfig.h>
#include <ConRIGv2/Radio/Radio.h>
#include <interfaces-impl/hwmapping.h>

using namespace std;
using namespace miosix;
using namespace Boardcore;
using namespace ConRIGv2;

Buttons::Buttons()
{
    resetState();
    state.arm_switch = false;
}

bool Buttons::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->getRadioScheduler();

    return scheduler.addTask([this]() { periodicStatusCheck(); },
                             Config::Buttons::BUTTON_SAMPLE_PERIOD) != 0;
}

mavlink_conrig_state_tc_t Buttons::getState() { return state; }

void Buttons::resetState()
{
    // Preserve the arm switch state
    auto armSwitch   = state.arm_switch;
    state            = {};
    state.arm_switch = armSwitch;
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
            LOG_DEBUG(logger, "Ignition button pressed");
        }
        else
        {
            guard++;
        }
    }
    else if (btns::ox_filling::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard                = 0;
            state.ox_filling_btn = true;
            LOG_DEBUG(logger, "Ox filling button pressed");
        }
        else
        {
            guard++;
        }
    }
    else if (btns::ox_release::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard                = 0;
            state.ox_release_btn = true;
            LOG_DEBUG(logger, "Ox release button pressed");
        }
        else
        {
            guard++;
        }
    }
    else if (btns::ox_detach::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard               = 0;
            state.ox_detach_btn = true;
            LOG_DEBUG(logger, "Ox detach button pressed");
        }
        else
        {
            guard++;
        }
    }
    else if (btns::n2_3way::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard               = 0;
            state.ox_detach_btn = true;
            LOG_DEBUG(logger, "n2 3way button pressed");
        }
        else
        {
            guard++;
        }
    }
    else if (btns::n2_filling::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard                = 0;
            state.n2_filling_btn = true;
            LOG_DEBUG(logger, "N2 filling button pressed");
        }
        else
        {
            guard++;
        }
    }
    else if (btns::n2_release::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard                = 0;
            state.n2_release_btn = true;
            LOG_DEBUG(logger, "N2 release button pressed");
        }
        else
        {
            guard++;
        }
    }
    else if (btns::n2_detach::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard               = 0;
            state.n2_detach_btn = true;
            LOG_DEBUG(logger, "N2 detach button pressed");
        }
        else
        {
            guard++;
        }
    }
    else if (btns::nitrogen::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard              = 0;
            state.nitrogen_btn = true;
            LOG_DEBUG(logger, "Nitrogen button pressed");
        }
        else
        {
            guard++;
        }
    }
    else if (btns::ox_venting::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard                = 0;
            state.ox_venting_btn = true;
            LOG_DEBUG(logger, "Ox venting button pressed");
        }
        else
        {
            guard++;
        }
    }
    else if (btns::n2_quenching::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard                  = 0;
            state.n2_quenching_btn = true;
            LOG_DEBUG(logger, "N2 quenching button pressed");
        }
        else
        {
            guard++;
        }
    }
    else if (btns::tars3::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard           = 0;
            state.tars3_btn = true;
            LOG_DEBUG(logger, "Tars3 button pressed");
        }
        else
        {
            guard++;
        }
    }
    else if (btns::tars3m::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard            = 0;
            state.tars3m_btn = true;
            LOG_DEBUG(logger, "Tars3m button pressed");
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
    getModule<Radio>()->setButtonsState(state);
}

void Buttons::enableIgnition() { ui::armedLed::high(); }

void Buttons::disableIgnition() { ui::armedLed::low(); }
