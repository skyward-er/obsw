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

#include <Payload/BoardScheduler.h>
#include <Payload/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Payload/Sensors/Sensors.h>
#include <Payload/StateMachines/ZVKController/ZVKControllerData.h>
#include <algorithms/ZVK/ZVK.h>
#include <diagnostic/PrintLogger.h>
#include <events/FSM.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Payload
{

class ZVKController
    : public Boardcore::FSM<ZVKController>,
      public Boardcore::InjectableWithDeps<BoardScheduler, Sensors,
                                           FlightStatsRecorder>
{
public:
    ZVKController();

    [[nodiscard]] bool start() override;

    void reset();

    ZVKControllerState getState();

    Boardcore::ZVKState getZVKState();

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

    uint64_t lastAccTimestamp0  = 0;
    uint64_t lastAccTimestamp1  = 0;
    uint64_t lastGyroTimestamp0 = 0;
    uint64_t lastGyroTimestamp1 = 0;
};

}  // namespace Payload

