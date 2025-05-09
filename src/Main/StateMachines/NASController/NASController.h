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
#include <Main/StateMachines/NASController/NASControllerData.h>
#include <Main/StatsRecorder/StatsRecorder.h>
#include <algorithms/NAS/NAS.h>
#include <diagnostic/PrintLogger.h>
#include <events/FSM.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Main
{

class NASController
    : public Boardcore::FSM<NASController>,
      public Boardcore::InjectableWithDeps<BoardScheduler, Sensors,
                                           StatsRecorder, AlgoReference>
{
public:
    NASController();

    [[nodiscard]] bool start() override;

    Boardcore::NASState getNASState();

    NASControllerState getState();

    /**
     * @brief Method to set the orientation of the system with the quaternions
     * (scalar first).
     */
    void setOrientation(Eigen::Quaternion<float> quat);

private:
    void update();
    void calibrate();

    // FSM states
    void state_init(const Boardcore::Event& event);
    void state_calibrating(const Boardcore::Event& event);
    void state_ready(const Boardcore::Event& event);
    void state_active(const Boardcore::Event& event);
    void state_end(const Boardcore::Event& event);

    void updateAndLogStatus(NASControllerState state);

    std::atomic<NASControllerState> state{NASControllerState::INIT};

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("nas");

    miosix::FastMutex nasMutex;
    Boardcore::NAS nas;

    int magDecimateCount  = 0;
    int acc1gSamplesCount = 0;
    bool acc1g            = false;

    uint64_t lastGyroTimestamp     = 0;
    uint64_t lastAccTimestamp      = 0;
    uint64_t lastMagTimestamp      = 0;
    uint64_t lastGpsTimestamp      = 0;
    uint64_t lastBaroTimestamp     = 0;
    uint64_t staticPitotTimestamp  = 0;
    uint64_t dynamicPitotTimestamp = 0;
};

}  // namespace Main
