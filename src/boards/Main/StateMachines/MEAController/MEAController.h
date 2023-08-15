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

#include <Main/StateMachines/MEAController/MEAControllerData.h>
#include <algorithms/MEA/MEA.h>
#include <diagnostic/PrintLogger.h>
#include <events/FSM.h>
#include <scheduler/TaskScheduler.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Main
{
class MEAController : public Boardcore::Module,
                      public Boardcore::FSM<MEAController>
{
public:
    MEAController(Boardcore::TaskScheduler* sched);

    /**
     * @brief Starts the FSM thread and adds an update function into the
     * scheduler.
     */
    bool start() override;

    /**
     * @brief Update function called periodically by the scheduler. It checks
     * the current FSM state and checks for engine shutdown.
     */
    void update();

    // MEA getters
    MEAControllerStatus getStatus();
    Boardcore::MEAState getMEAState();

    // FSM states
    void state_idle(const Boardcore::Event& event);
    void state_ready(const Boardcore::Event& event);
    void state_armed(const Boardcore::Event& event);
    void state_shadow_mode(const Boardcore::Event& event);
    void state_active(const Boardcore::Event& event);
    void state_end(const Boardcore::Event& event);

private:
    /**
     * @brief Logs the MEA status updating the FSM state
     * @param state The current FSM state
     */
    void logStatus(MEAControllerState state);

    // TODO comment
    Boardcore::MEA::KalmanFilter::KalmanConfig getMEAKalmanConfig();

    // Controller state machine status
    MEAControllerStatus status;
    Boardcore::MEA mea;

    // Scheduler to be used for update function
    Boardcore::TaskScheduler* scheduler = nullptr;

    // Thread safe counter for engine shutdown
    std::atomic<uint16_t> detectedShutdowns{0};

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("MEA");
};
}  // namespace Main