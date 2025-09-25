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
    enum class ADANumber : uint8_t
    {
        ADA0 = 0,
        ADA1 = 1,
        ADA2 = 2,
    };

    ADAController();

    [[nodiscard]] bool start() override;

    Boardcore::ADAState getADAState(ADANumber num);

    ADAControllerState getState();

    float getDeploymentAltitude();

    /**
     * @brief Returns the maximum vertical speed (in module) of the ADAs.
     */
    float getMaxVerticalSpeed();

    /**
     * @brief Returns the maximum pressure (in module) of the ADAs.
     */
    float getMaxPressure();

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

    miosix::FastMutex adaMutex;
    Boardcore::ADA ada0;
    Boardcore::ADA ada1;
    Boardcore::ADA ada2;

    uint64_t lastBaro0Timestamp = 0;
    uint64_t lastBaro1Timestamp = 0;
    uint64_t lastBaro2Timestamp = 0;

    uint32_t ada0DetectedApogees = 0;
    uint32_t ada1DetectedApogees = 0;
    uint32_t ada2DetectedApogees = 0;
    // Bitset to track whether each ADA has detected an apogee
    std::bitset<3> apogeeDetections;

    uint32_t ada0DetectedDeployments = 0;
    uint32_t ada1DetectedDeployments = 0;
    uint32_t ada2DetectedDeployments = 0;
    // Bitset to track whether each ADA has detected a deployment
    std::bitset<3> deploymentDetections;
};

}  // namespace Main
