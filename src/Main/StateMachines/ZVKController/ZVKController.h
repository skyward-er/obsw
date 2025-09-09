/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Giovanni Annaloro
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
#include <Main/StateMachines/ZVKController/ZVKControllerData.h>
#include <Main/StatsRecorder/StatsRecorder.h>
#include <algorithms/ZVK/ZVK.h>
#include <diagnostic/PrintLogger.h>
#include <events/FSM.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Main
{

class ZVKController
    : public Boardcore::FSM<ZVKController>,
      public Boardcore::InjectableWithDeps<BoardScheduler, Sensors,
                                           StatsRecorder, AlgoReference>
{
public:
    ZVKController();

    [[nodiscard]] bool start() override;

    ZVKControllerState getState();

    Boardcore::ZVKState getZVKState();

    /**
     * @brief Method to set the orientation of the system with the quaternions
     * (scalar last).
     */
    void setOrientation(Eigen::Quaternion<float> quat);

private:
    void update();

    // FSM states
    void state_init(const Boardcore::Event& event);
    void state_active(const Boardcore::Event& event);
    void state_end(const Boardcore::Event& event);

    void updateAndLogStatus(ZVKControllerState state);

    std::atomic<ZVKControllerState> state{ZVKControllerState::INIT};

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("zvk");

    miosix::FastMutex zvkMutex;
    Boardcore::ZVK zvk;

    // int magDecimateCount  = 0;
    // int acc1gSamplesCount = 0;
    // bool acc1g            = false;

    uint64_t lastGyroTimestamp = 0;
    uint64_t lastAccTimestamp  = 0;
    uint64_t lastMagTimestamp  = 0;
};

}  // namespace Main

