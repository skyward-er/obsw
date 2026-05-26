/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Tommaso Lamon
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
#include <Main/StateMachines/SDAController/SDAControllerData.h>
#include <algorithms/SDA/Kriging0.h>
#include <events/FSM.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Main
{

class SDAController : public Boardcore::FSM<SDAController>,
                      public Boardcore::InjectableWithDeps<BoardScheduler>
{
public:
    SDAController();

    virtual ~SDAController() noexcept = default;
    [[nodiscard]] bool start() override;

    SDAControllerState getControllerState();

private:
    void update();
    void calibrate();  // check

    void updateAndLogStatus(SDAControllerState state);

    miosix::FastMutex sdaMutex;

    void state_init(const Boardcore::Event& event);
    void state_calibrating(const Boardcore::Event& event);  // check
    void state_ready(const Boardcore::Event& event);
    void state_armed(const Boardcore::Event& event);
    void state_powered_ascent(const Boardcore::Event& event);
    void state_unpowered_ascent(const Boardcore::Event& event);

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sda");

    std::atomic<SDAControllerState> state{SDAControllerState::INIT};

    Kriging0 sda;
    size_t sdaID;
};

}  // namespace Main
