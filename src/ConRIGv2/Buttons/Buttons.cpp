/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Ettore Pane, Niccolò Betto
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

bool Buttons::start()
{
    TaskScheduler& scheduler =
        getModule<BoardScheduler>()->getButtonsScheduler();

    return scheduler.addTask([this]() { periodicStatusCheck(); },
                             Config::Buttons::BUTTON_SAMPLE_PERIOD) != 0;
}

mavlink_conrig_state_tc_t Buttons::getState() { return state; }

void Buttons::periodicStatusCheck()
{
#define CHECK_BUTTON(cond, btn)                           \
    if (cond)                                             \
    {                                                     \
        if (guard.btn > Config::Buttons::GUARD_THRESHOLD) \
        {                                                 \
            guard.btn = 0;                                \
            state.btn = true;                             \
            LOG_DEBUG(logger, #btn " button pressed");    \
        }                                                 \
        else                                              \
        {                                                 \
            guard.btn++;                                  \
        }                                                 \
    }                                                     \
    else                                                  \
    {                                                     \
        guard.btn = 0;                                    \
        state.btn = false;                                \
    }

    state.arm_switch  = btns::arm::value();
    state.n2_3way_btn = btns::n2_3way::value();

    CHECK_BUTTON(!btns::ignition::value() && state.arm_switch, ignition_btn);
    CHECK_BUTTON(btns::ox_filling::value(), ox_filling_btn);
    CHECK_BUTTON(btns::ox_release::value(), ox_release_btn);
    CHECK_BUTTON(btns::ox_detach::value(), ox_detach_btn);
    CHECK_BUTTON(btns::ox_venting::value(), ox_venting_btn);
    CHECK_BUTTON(btns::n2_filling::value(), n2_filling_btn);
    CHECK_BUTTON(btns::n2_release::value(), n2_release_btn);
    CHECK_BUTTON(btns::n2_detach::value(), n2_detach_btn);
    CHECK_BUTTON(btns::n2_quenching::value(), n2_quenching_btn);
    CHECK_BUTTON(btns::nitrogen::value(), nitrogen_btn);
    CHECK_BUTTON(btns::tars3::value(), tars3_btn);
    CHECK_BUTTON(btns::tars3m::value(), tars3m_btn);

#undef CHECK_BUTTON

    // Set the internal button state in Radio module
    getModule<Radio>()->updateButtonState(state);
}

void Buttons::enableIgnition() { ui::armedLed::high(); }

void Buttons::disableIgnition() { ui::armedLed::low(); }
