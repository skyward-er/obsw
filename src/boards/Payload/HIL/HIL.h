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

#include <Payload/Actuators/Actuators.h>
#include <Payload/Buses.h>
#include <Payload/Configs/HILSimulationConfig.h>
#include <Payload/HIL/HILData.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Payload/StateMachines/NASController/NASController.h>
#include <Payload/StateMachines/WingController/WingController.h>
#include <Payload/WindEstimationScheme/WindEstimation.h>
#include <common/Events.h>
#include <hil/HIL.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Payload
{

class PayloadHILTransceiver
    : public Boardcore::HILTransceiver<PayloadFlightPhases, SimulatorData,
                                       ActuatorData>
{
    using Boardcore::HILTransceiver<PayloadFlightPhases, SimulatorData,
                                    ActuatorData>::HILTransceiver;
};

class PayloadHILPhasesManager
    : public Boardcore::HILPhasesManager<PayloadFlightPhases, SimulatorData,
                                         ActuatorData>
{
public:
    explicit PayloadHILPhasesManager(
        std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition);

    void processFlagsImpl(
        const SimulatorData& simulatorData,
        std::vector<PayloadFlightPhases>& changed_flags) override;

    void printOutcomes();

private:
    void handleEventImpl(
        const Boardcore::Event& e,
        std::vector<PayloadFlightPhases>& changed_flags) override;
};

class PayloadHIL
    : public Boardcore::HIL<PayloadFlightPhases, SimulatorData, ActuatorData>,
      public Boardcore::InjectableWithDeps<
          Payload::Buses, Payload::Actuators, Payload::FlightModeManager,
          Payload::WindEstimation, Payload::WingController,
          Payload::NASController>
{

public:
    PayloadHIL();

    bool start() override;

private:
    ActuatorData updateActuatorData();
};

}  // namespace Payload