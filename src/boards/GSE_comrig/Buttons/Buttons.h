/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <diagnostic/PrintLogger.h>
#include <common/Mavlink.h>
#include <utils/ModuleManager/ModuleManager.hpp>

namespace GSE_comrig
{

struct ButtonsState{
    bool ignition;
    bool fillin_valve;
    bool venting_valve;
    bool release_filling_line_pressure;
    bool detach_quick_connector;
    bool startup_tars;

    // FIXME: mavlink_send_comrig_state_tc_t
    // mavlink_send_comrig_state_tc_t buttonsToMavlink()
    // {
    //     mavlink_send_comrig_state_tc_t tc = {};
    //     tc.ignition_btn = ignition;
    //     tc.filling_valve_btn = fillin_valve;
    //     tc.venting_valve_btn = venting_valve;
    //     tc.release_pressure_btn = release_filling_line_pressure;
    //     tc.quick_connector_btn = detach_quick_connector;
    //     tc.start_tars_btn = startup_tars;

    //     return tc;
    // }
};

class Buttons : public Boardcore::Module
{

public:
    Buttons();

    ~Buttons();

    bool start() { return true; };

    bool isStarted() { return true; };

    ButtonsState getState();
    void resetState();
    int shouldArm();

private:
    ButtonsState state;
    bool isArmed;
    bool wasArmed;
    void periodicStatusCheck();

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("buttons");
};

}  // namespace GSE_comrig
