/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include <RIG/StateMachines/GroundModeManager/GroundModeManagerData.h>
#include <events/EventBroker.h>
#include <events/FSM.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace RIG
{
class GroundModeManager : public Boardcore::Module,
                          public Boardcore::FSM<GroundModeManager>
{
public:
    GroundModeManager();

    // FSM states
    void state_idle(const Boardcore::Event& event);
    void state_ready(const Boardcore::Event& event);
    void state_armed(const Boardcore::Event& event);
    void state_igniting(const Boardcore::Event& event);

    /**
     * @brief Sets the ignition time
     *
     * @param ignitionTime Time in [ms]
     */
    void setIgnitionTime(uint32_t ignitionTime);

    // Status getter
    GroundModeManagerStatus getStatus();

private:
    /**
     * @brief Logs the current FSM status on SD
     */
    void logStatus(GroundModeManagerState state);

    // User Radio set ignition time
    uint32_t ignitionTime;

    // Current status
    GroundModeManagerStatus status;

    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("GroundModeManager");
};

}  // namespace RIG