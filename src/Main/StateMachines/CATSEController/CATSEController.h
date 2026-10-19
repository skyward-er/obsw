/* Copyright (c) 2026 Skyward Experimental Rocketry
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

#include <Main/BoardScheduler.h>
#include <Main/Configs/CATSEConfig.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/CATSEController/CATSEControllerData.h>
#include <anas/cats/CATS.h>
#include <diagnostic/PrintLogger.h>
#include <events/FSM.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <chrono>

namespace Main
{

class CATSEController
    : public Boardcore::FSM<CATSEController>,
      public Boardcore::InjectableWithDeps<BoardScheduler, Sensors>
{
public:
    CATSEController();

    [[nodiscard]] bool start() override;

    Boardcore::CATSEState getCATSEState();

    CATSEControllerState getState();

private:
    void update();

    // FSM states
    void state_init(const Boardcore::Event& event);
    void state_running(const Boardcore::Event& event);
    void state_end(const Boardcore::Event& event);

    void updateAndLogStatus(CATSEControllerState state);

    std::atomic<CATSEControllerState> state{CATSEControllerState::INIT};

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sda");

    Boardcore::CATSEState catseState{};

    miosix::FastMutex catseMutex;
    CATS cats;
};

}  // namespace Main
