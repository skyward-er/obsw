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

#include <Main/StateMachines/ABKController/ABKControllerData.h>
#include <algorithms/AirBrakes/AirBrakesInterp.h>
#include <events/FSM.h>
#include <scheduler/TaskScheduler.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Main
{
class ABKController : public Boardcore::Module,
                      public Boardcore::FSM<ABKController>
{
public:
    ABKController(Boardcore::TaskScheduler* sched);

    /**
     * @brief Starts the update task and the FSM
     */
    bool start() override;

    /**
     * @brief Periodic called by the Task scheduler, updates the ABKInterp
     * algorithm
     */
    void update();

    /**
     * @brief Returns the ABK FSM status along with the last update timestamp
     */
    ABKControllerStatus getStatus();

    // FSM states
    void state_init(const Boardcore::Event& event);
    void state_idle(const Boardcore::Event& event);
    void state_active(const Boardcore::Event& event);
    void state_end(const Boardcore::Event& event);

private:
    /**
     * @brief Updates the internal status and logs it
     */
    void logStatus(ABKControllerState state);

    /**
     * @brief Returns the default config applied to the ABK algorithm
     */
    Boardcore::AirBrakesInterpConfig getConfig();

    Boardcore::AirBrakesInterp abk;

    ABKControllerStatus status;
    Boardcore::TaskScheduler* scheduler = nullptr;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("ABK");
};
}  // namespace Main