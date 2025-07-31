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

#include <Main/Actuators/Actuators.h>
#include <Main/Buses.h>
#include <Main/Configs/HILSimulationConfig.h>
#include <Main/HIL/HILData.h>
#include <Main/StateMachines/ABKController/ABKController.h>
#include <Main/StateMachines/ADAController/ADAController.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Main/StateMachines/MEAController/MEAController.h>
#include <Main/StateMachines/NASController/NASController.h>
#include <common/Events.h>
#include <common/canbus/MotorStatus.h>
#include <hil/HIL.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Main
{

class MainHILTransceiver
    : public Boardcore::HILTransceiver<MainFlightPhases, SimulatorData,
                                       ActuatorData>
{
    using Boardcore::HILTransceiver<MainFlightPhases, SimulatorData,
                                    ActuatorData>::HILTransceiver;
};

class MainHILPhasesManager
    : public Boardcore::HILPhasesManager<MainFlightPhases, SimulatorData,
                                         ActuatorData>
{
public:
    explicit MainHILPhasesManager(
        std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition);

    void processFlagsImpl(
        const SimulatorData& simulatorData,
        std::vector<MainFlightPhases>& changed_flags) override;

    void printOutcomes();

private:
    void handleEventImpl(const Boardcore::Event& e,
                         std::vector<MainFlightPhases>& changed_flags) override;
};

class MainHIL
    : public Boardcore::HIL<MainFlightPhases, SimulatorData, ActuatorData>,
      public Boardcore::InjectableWithDeps<
          Buses, Actuators, FlightModeManager, ADAController, NASController,
          MEAController, ABKController, Common::MotorStatus>
{
public:
    MainHIL();

    bool start() override;

private:
    ActuatorData updateActuatorData();
};

}  // namespace Main
