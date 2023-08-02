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

#include <algorithms/NAS/NAS.h>
#include <diagnostic/PrintLogger.h>
#include <events/FSM.h>
#include <scheduler/TaskScheduler.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#include "NASControllerData.h"

namespace Payload
{
class NASController : public Boardcore::FSM<NASController>,
                      public Boardcore::Module
{
public:
    NASController(Boardcore::TaskScheduler* sched);

    // Starts the FSM thread and adds the update function into the scheduler
    bool start() override;

    // NAS FSM called methods
    void calibrate();

    // NAS setters
    void setCoordinates(Eigen::Vector2f position);
    void setOrientation(float yaw, float pitch, float roll);
    void setReferenceAltitude(float altitude);
    void setReferenceTemperature(float temperature);
    void setReferenceValues(const Boardcore::ReferenceValues reference);

    // NAS Getters
    NASControllerStatus getStatus();
    Boardcore::NASState getNasState();
    Boardcore::ReferenceValues getReferenceValues();

    // FSM states
    void state_idle(const Boardcore::Event& event);
    void state_calibrating(const Boardcore::Event& event);
    void state_ready(const Boardcore::Event& event);
    void state_active(const Boardcore::Event& event);
    void state_end(const Boardcore::Event& event);

private:
    void update();
    /**
     * @brief Logs the NAS status updating the FSM state
     * @param state The current FSM state
     */
    void logStatus(NASControllerState state);

    // Controller state machine status
    NASControllerStatus status;
    Boardcore::NAS nas;

    // Scheduler to be used for update function
    Boardcore::TaskScheduler* scheduler;

    // User set (or triac set) initial orientation
    Eigen::Vector3f initialOrientation;
    bool accelerationValid = true;

    u_int8_t accSampleAfterSpike = 0;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("NAS");
};
}  // namespace Payload
