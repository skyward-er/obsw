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

#include <RIGv2/Actuators/Actuators.h>
#include <RIGv2/BoardScheduler.h>
#include <RIGv2/Configs/ERegControllerConfig.h>
#include <RIGv2/StateMachines/ERegController/ERegControllerData.h>
#include <algorithms/EReg/EReg.h>
#include <algorithms/EReg/ERegData.h>
#include <common/MedianFilter.h>
#include <events/FSM.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace RIGv2
{

class ERegControllerOx
    : public Boardcore::FSM<ERegControllerOx>,
      public Boardcore::InjectableWithDeps<BoardScheduler, Actuators, Sensors>
{
public:
    ERegControllerOx();

    [[nodiscard]] bool start() override;

    ERegState getState();

    void changePIDConfig(Boardcore::ERegPIDConfig newPressurizationConfig,
                         Boardcore::ERegPIDConfig newDischargeConfig);

    void changeTargetPressure(float newTargetPressure);
    void setIntegralContribution(float newPilotContribution,
                                 float newRampupContribution);

private:
    void update();

    // FSM states
    void state_init(const Boardcore::Event& event);
    void state_closed(const Boardcore::Event& event);
    void state_pressurizing(const Boardcore::Event& event);
    void state_pilot_flame(const Boardcore::Event& event);
    void state_rampup(const Boardcore::Event& event);

    void updateAndLogStatus(ERegState state);

    std::atomic<ERegState> state{ERegState::INIT};

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("ereg");

    Boardcore::EReg regulator;
    MedianFilter<float, Config::ERegOx::MEDIAN_SAMPLE_NUMBER>
        downstreamPressureFilter;
    MedianFilter<float, Config::ERegOx::MEDIAN_SAMPLE_NUMBER>
        upstreamPressureFilter;

    float lastDownstreamInput;  // Last sample used by ereg algorithm
    float lastUpstreamInput;    // Last sample used by ereg algorithm

    Boardcore::ERegPIDConfig pressurizationConfig =
        Config::ERegOx::STABILIZING_CONFIG;
    Boardcore::ERegPIDConfig dischargeConfig =
        Config::ERegOx::DISCHARGING_CONFIG;

    float targetPressure     = Config::ERegOx::TARGET_PRESSURE;
    float pilotFlameIntegral = Config::ERegOx::PILOT_FLAME_INTEGRAL;
    float rampupIntegral     = Config::ERegOx::RAMPUP_INTEGRAL;
};

}  // namespace RIGv2
