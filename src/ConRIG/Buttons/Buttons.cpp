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

#include <ConRIG/BoardScheduler.h>
#include <ConRIG/Configs/ButtonsConfig.h>
#include <ConRIG/Radio/Radio.h>
#include <interfaces-impl/hwmapping.h>

using namespace std;
using namespace miosix;
using namespace Boardcore;
using namespace ConRIG;

Buttons::Buttons()
{
    resetState();
    state.arm_switch = false;
}

bool Buttons::start()
{
    TaskScheduler &scheduler = getModule<BoardScheduler>()->getRadioScheduler();

    return scheduler.addTask([this]() { periodicStatusCheck(); },
                             Config::Buttons::BUTTON_SAMPLE_PERIOD) != 0;
}

mavlink_conrig_state_tc_t Buttons::getState() { return state; }

void Buttons::resetState()
{
    state.n2o_filling_btn  = false;
    state.n2o_release_btn  = false;
    state.n2_filling_btn   = false;
    state.n2_release_btn   = false;
    state.n2_detach_btn    = false;
    state.n2o_venting_btn  = false;
    state.nitrogen_btn     = false;
    state.n2o_detach_btn   = false;
    state.n2_quenching_btn = false;
    state.n2_3way_btn      = false;
    state.tars_btn         = false;
    state.ignition_btn     = false;
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
    else if (btns::n2o_filling::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard                 = 0;
            state.n2o_filling_btn = true;
            LOG_DEBUG(logger, "n2o filling button pressed");
        }
        else
        {
            guard++;
        }
    }
    else if (btns::n2o_release::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard                 = 0;
            state.n2o_release_btn = true;
            LOG_DEBUG(logger, "n2o release button pressed");
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
            LOG_DEBUG(logger, "n2 release button pressed");
        }
        else
        {
            guard++;
        }
    }
    else if (btns::n2o_venting::value())
    {
        if (guard > Config::Buttons::GUARD_THRESHOLD)
        {
            guard                 = 0;
            state.n2o_venting_btn = true;
            LOG_DEBUG(logger, "n2o venting button pressed");
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
            LOG_DEBUG(logger, "n2 detach button pressed");
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
            LOG_DEBUG(logger, "n2 filling button pressed");
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
            LOG_DEBUG(logger, "nitrogen button pressed");
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
