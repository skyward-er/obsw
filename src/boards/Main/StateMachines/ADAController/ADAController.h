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

#include <Main/StateMachines/ADAController/ADAControllerData.h>
#include <algorithms/ADA/ADA.h>
#include <diagnostic/PrintLogger.h>
#include <events/FSM.h>
#include <scheduler/TaskScheduler.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Main
{
class ADAController : public Boardcore::Module,
                      public Boardcore::FSM<ADAController>
{
public:
    ADAController(Boardcore::TaskScheduler* sched);

    /**
     * @brief Starts the FSM thread and adds an update function into the
     * scheduler
     */
    bool start() override;

    /**
     * @brief Update function called periodically by the scheduler. It checks
     * the current FSM state and checks for apogees.
     */
    void update();

    void calibrate();

    // ADA setters
    void setReferenceAltitude(float altitude);
    void setReferenceTemperature(float temperature);
    void setReferenceValues(const Boardcore::ReferenceValues reference);

    // ADA getters
    ADAControllerStatus getStatus();
    Boardcore::ADAState getADAState();
    Boardcore::ReferenceValues getReferenceValues();

    // FSM states
    void state_idle(const Boardcore::Event& event);
    void state_calibrating(const Boardcore::Event& event);
    void state_ready(const Boardcore::Event& event);
    void state_shadow_mode(const Boardcore::Event& event);
    void state_active(const Boardcore::Event& event);
    void state_end(const Boardcore::Event& event);

private:
    /**
     * @brief Logs the ADA status updating the FSM state
     * @param state The current FSM state
     */
    void logStatus(ADAControllerState state);

    // TODO comment
    Boardcore::ADA::KalmanFilter::KalmanConfig getADAKalmanConfig();

    // Controller state machine status
    ADAControllerStatus status;
    Boardcore::ADA ada;

    // Scheduler to be used for update function
    Boardcore::TaskScheduler* scheduler = nullptr;

    // Counter that keeps trace of the number of apogees
    uint16_t detectedApogeeEvents = 0;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("ADA");
};
}  // namespace Main