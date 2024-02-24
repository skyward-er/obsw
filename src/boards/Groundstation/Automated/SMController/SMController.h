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

#include <events/EventBroker.h>
#include <events/HSM.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#include "SMControllerData.h"

namespace Antennas
{

class SMController : public Boardcore::Module,
                     public Boardcore::HSM<SMController>
{
public:
    SMController();

    // FSM States
    Boardcore::State state_feedback(const Boardcore::Event& event);
    Boardcore::State state_no_feedback(const Boardcore::Event& event);
    Boardcore::State state_init(const Boardcore::Event& event);
    Boardcore::State state_init_error(const Boardcore::Event& event);
    Boardcore::State state_init_done(const Boardcore::Event& event);
    Boardcore::State insert_info(const Boardcore::Event& event);
    Boardcore::State state_armed(const Boardcore::Event& event);
    Boardcore::State state_test(const Boardcore::Event& event);
    Boardcore::State state_calibrate(const Boardcore::Event& event);
    Boardcore::State state_fix_antennas(const Boardcore::Event& event);
    Boardcore::State state_fix_rocket(const Boardcore::Event& event);
    Boardcore::State state_active(const Boardcore::Event& event);
    Boardcore::State state_armed_nf(const Boardcore::Event& event);
    Boardcore::State state_test_nf(const Boardcore::Event& event);
    Boardcore::State state_fix_rocket_nf(const Boardcore::Event& event);
    Boardcore::State state_active_nf(const Boardcore::Event& event);

private:
    /**
     * @brief Logs the current state of the FSM
     * @param state The current FSM state
     */
    void logStatus(SMControllerState state);

    SMControllerStatus status;

    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("SMController");
};

}  // namespace Antennas
