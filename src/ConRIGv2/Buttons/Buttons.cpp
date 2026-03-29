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

#include <iomanip>
#include <iostream>

using namespace std;
using namespace miosix;
using namespace Boardcore;
using namespace ConRIGv2;

void printStateDiff(const mavlink_conrig_state_tc_t& oldState,
                    const mavlink_conrig_state_tc_t& state)
{
#define BUTTON(btn)                                                        \
    std::setw(8) << std::right << (state.btn != oldState.btn ? "*  " : "") \
                 << std::setw(20) << std::left << #btn << std::setw(5)     \
                 << std::left << (state.btn ? "On" : "Off") << std::left   \
                 << "(" << (int)state.btn << ")"

    std::cout << "Button state changed: \n"
              << BUTTON(ox_filling_btn) << "\n"
              << BUTTON(ox_release_btn) << "\n"
              << BUTTON(prz_filling_btn) << "\n"
              << BUTTON(prz_release_btn) << "\n"
              << BUTTON(prz_ox_btn) << "\n"
              << BUTTON(prz_fuel_btn) << "\n"
              << BUTTON(ox_venting_btn) << "\n"
              << BUTTON(detach_btn) << "\n"
              << BUTTON(spare_0_btn) << "\n"
              << BUTTON(spare_1_btn) << "\n"
              << BUTTON(spare_2_btn) << "\n"
              << BUTTON(spare_3_btn) << "\n"
              << BUTTON(spare_4_btn) << "\n"
              << BUTTON(spare_5_btn) << "\n"
              << BUTTON(arm_switch) << "\n"
              << BUTTON(prz_3way_switch) << "\n"
              << BUTTON(tars_switch) << "\n"
              << BUTTON(ignition_btn) << "\n";
#undef BUTTON
}

bool Buttons::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->buttons();

    return scheduler.addTask([this]() { periodicStatusCheck(); },
                             Config::Buttons::BUTTON_SAMPLE_PERIOD) != 0;
}

mavlink_conrig_state_tc_t Buttons::getState() { return state; }

void Buttons::periodicStatusCheck()
{
    auto oldState = state;
    (void)oldState;  // Avoid unused variable warning

#define CHECK_BUTTON(cond, btn)                           \
    if (cond)                                             \
    {                                                     \
        if (guard.btn > Config::Buttons::GUARD_THRESHOLD) \
        {                                                 \
            guard.btn = 0;                                \
            state.btn = true;                             \
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

    // Handle switches (levers)
    state.arm_switch      = btns::arm::value();
    state.prz_3way_switch = btns::prz_3way::value();

    // The tars switch has 2 position that close the circuit on different pins
    // We still want to ensure only one can be active at any time via software
    // If for some reason both TARS pins are high, reuse the old state because
    // in the case of TARS running, setting it to OFF would stop it
    // By keeping the old state we ensure that TARS is not stopped in case of
    // interference on the buttons (rare case anyway)
    if (btns::tars1::value() && btns::tars3::value())
        state.tars_switch = oldState.tars_switch;
    else if (btns::tars1::value())
        state.tars_switch = TARSList::TARS_1;
    else if (btns::tars3::value())
        state.tars_switch = TARSList::TARS_3;
    else
        state.tars_switch = TARSList::TARS_OFF;

    // The ignition button is considered only if the arm switch is active
    CHECK_BUTTON(!btns::ignition::value() && state.arm_switch, ignition_btn);

    CHECK_BUTTON(btns::ox_filling::value(), ox_filling_btn);
    CHECK_BUTTON(btns::ox_release::value(), ox_release_btn);
    CHECK_BUTTON(btns::prz_filling::value(), prz_filling_btn);
    CHECK_BUTTON(btns::prz_release::value(), prz_release_btn);
    CHECK_BUTTON(btns::prz_ox::value(), prz_ox_btn);
    CHECK_BUTTON(btns::prz_fuel::value(), prz_fuel_btn);
    CHECK_BUTTON(btns::ox_venting::value(), ox_venting_btn);
    CHECK_BUTTON(btns::detach::value(), detach_btn);
    CHECK_BUTTON(!btns::spare_0::value(), spare_0_btn);
    CHECK_BUTTON(btns::spare_1::value(), spare_1_btn);
    CHECK_BUTTON(btns::spare_2::value(), spare_2_btn);
    CHECK_BUTTON(!btns::spare_3::value(), spare_3_btn);
    CHECK_BUTTON(!btns::spare_4::value(), spare_4_btn);
    CHECK_BUTTON(!btns::spare_5::value(), spare_5_btn);

#undef CHECK_BUTTON

    // Set the internal button state in Radio module
    getModule<Radio>()->updateButtonState(state);

#ifndef NDEBUG
    // Debug print
    if (std::memcmp(&oldState, &state, sizeof(state)) != 0)
        printStateDiff(oldState, state);
#endif
}

void Buttons::enableIgnition() { ui::armedLed::high(); }

void Buttons::disableIgnition() { ui::armedLed::low(); }
