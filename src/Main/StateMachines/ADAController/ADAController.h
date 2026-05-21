/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <Main/AlgoReference/AlgoReference.h>
#include <Main/BoardScheduler.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/ADAController/ADAControllerData.h>
#include <Main/StatsRecorder/StatsRecorder.h>
#include <algorithms/ADA/ADA.h>
#include <events/FSM.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <bitset>

namespace Main
{

class ADAController
    : public Boardcore::InjectableWithDeps<BoardScheduler, Sensors,
                                           StatsRecorder, AlgoReference>,
      public Boardcore::FSM<ADAController>,
      public ReferenceSubscriber
{
public:
    ADAController();

    [[nodiscard]] bool start() override;

    Boardcore::ADAState getADAState();

    ADAControllerState getState();

    float getDeploymentAltitude();
    void setDeploymentAltitude(float altitude);

    std::chrono::milliseconds getShadowModeTime();
    void setShadowModeTime(std::chrono::milliseconds time);

    void onReferenceChanged(const Boardcore::ReferenceValues& ref) override;

private:
    void update();
    void calibrate();

    // FSM states
    void state_init(const Boardcore::Event& event);
    void state_calibrating(const Boardcore::Event& event);
    void state_ready(const Boardcore::Event& event);
    void state_armed(const Boardcore::Event& event);
    void state_shadow_mode(const Boardcore::Event& event);
    void state_active_ascent(const Boardcore::Event& event);
    void state_active_drogue_descent(const Boardcore::Event& event);
    void state_active_terminal_descent(const Boardcore::Event& event);
    void state_end(const Boardcore::Event& event);

    void updateAndLogStatus(ADAControllerState state);

    std::atomic<ADAControllerState> state{ADAControllerState::INIT};

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("ada");

    uint16_t shadowModeTimeoutEvent = 0;

    std::atomic<float> deploymentAltitude;  // [m]
    std::atomic<std::chrono::milliseconds> shadowModeTime;

    miosix::FastMutex adaMutex;
    Boardcore::ADA ada;

    uint64_t lastBaroTimestamp = 0;

    uint32_t adaDetectedApogees = 0;
    bool apogeeDetection;

    uint32_t adaDetectedDeployments = 0;
    bool deploymentDetections;
};

}  // namespace Main
