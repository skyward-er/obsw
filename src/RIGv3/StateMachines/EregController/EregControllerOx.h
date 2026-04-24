/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Pietro Bortolus
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

#include <RIGv3/Actuators/Actuators.h>
#include <RIGv3/BoardScheduler.h>
#include <RIGv3/Configs/EregControllerConfig.h>
#include <RIGv3/StateMachines/EregController/EregControllerData.h>
#include <algorithms/Ereg/Ereg.h>
#include <algorithms/Ereg/EregData.h>
#include <common/MeanFilter.h>
#include <events/FSM.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace RIGv3
{

class EregControllerOx
    : public Boardcore::FSM<EregControllerOx>,
      public Boardcore::InjectableWithDeps<BoardScheduler, Registry, Actuators,
                                           Sensors>
{
public:
    EregControllerOx();

    [[nodiscard]] bool start() override;

    EregState getState();

    void changePIDConfig(Boardcore::EregPIDConfig newPressurizationConfig,
                         Boardcore::EregPIDConfig newDischargeConfig);

    void changeTargetPressure(float newFirstPressurizationTargetPressure,
                              float newRampupTargetPressure);
    void setIntegralPrecharge(float newPilotPrecharge,
                              float newRampupPrecharge);

private:
    void update();
    void loadFromRegistry();

    // FSM states
    void state_init(const Boardcore::Event& event);
    void state_closed(const Boardcore::Event& event);
    void state_pressurizing(const Boardcore::Event& event);
    void state_pilot_flame(const Boardcore::Event& event);
    void state_firing(const Boardcore::Event& event);

    void updateAndLogStatus(EregState state);

    std::atomic<EregState> state{EregState::INIT};

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("ereg");

    Boardcore::Ereg regulator;
    Common::MeanFilter<float, Config::EregOx::FILTER_SAMPLES>
        downstreamPressureFilter;
    Common::MeanFilter<float, Config::EregOx::FILTER_SAMPLES>
        upstreamPressureFilter;

    float lastDownstreamInput = 0.0f;
    float lastUpstreamInput   = 0.0f;

    Boardcore::EregPIDConfig pressurizationConfig =
        Config::EregOx::STABILIZING_CONFIG;
    Boardcore::EregPIDConfig dischargeConfig =
        Config::EregOx::DISCHARGING_CONFIG;

    float firstPressurizationTargetPressure =
        Config::EregOx::FIRST_PRESSURIZATION_TARGET_PRESSURE;
    float rampupTargetPressure = Config::EregOx::RAMPUP_TARGET_PRESSURE;

    float pilotFlamePrecharge = Config::EregOx::PILOT_FLAME_PRECHARGE;
    float rampupPrecharge     = Config::EregOx::RAMPUP_PRECHARGE;
};

}  // namespace RIGv3
