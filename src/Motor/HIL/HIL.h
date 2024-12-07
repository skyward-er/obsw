/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <Motor/Actuators/Actuators.h>
#include <Motor/Buses.h>
#include <Motor/Configs/HILSimulationConfig.h>
#include <common/Events.h>
#include <events/EventBroker.h>
#include <hil/HIL.h>

#include "HILData.h"

namespace Motor
{

class MotorHILTransceiver
    : public Boardcore::HILTransceiver<MotorFlightPhases, SimulatorData,
                                       ActuatorData>
{
    using Boardcore::HILTransceiver<MotorFlightPhases, SimulatorData,
                                    ActuatorData>::HILTransceiver;
};

class MotorHILPhasesManager
    : public Boardcore::HILPhasesManager<MotorFlightPhases, SimulatorData,
                                         ActuatorData>
{
public:
    explicit MotorHILPhasesManager(
        std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition);

    void processFlagsImpl(
        const SimulatorData& simulatorData,
        std::vector<MotorFlightPhases>& changed_flags) override;

    void printOutcomes();

private:
    void handleEventImpl(
        const Boardcore::Event& e,
        std::vector<MotorFlightPhases>& changed_flags) override;
};

class MotorHIL
    : public Boardcore::HIL<MotorFlightPhases, SimulatorData, ActuatorData>,
      public Boardcore::InjectableWithDeps<Buses, Actuators>

{
public:
    MotorHIL();

    bool start() override;

private:
    ActuatorData updateActuatorData();
};
}  // namespace Motor
