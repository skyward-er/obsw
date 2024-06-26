/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Giacomo Caironi
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

#include <common/MavlinkGemini.h>
#include <diagnostic/PrintLogger.h>
#include <scheduler/TaskScheduler.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace con_RIG
{

class Buttons : public Boardcore::Module
{

public:
    explicit Buttons(Boardcore::TaskScheduler* sched);

    ~Buttons();

    bool start();

    mavlink_conrig_state_tc_t getState();

    void resetState();

    void setRemoteArmState(int state);

private:
    mavlink_conrig_state_tc_t state;
    void periodicStatusCheck();
    std::atomic<bool> remoteArm{false};

    // Counter guard to avoid spurious triggers
    uint8_t guard = 0;

    Boardcore::TaskScheduler* scheduler = nullptr;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("buttons");
};

}  // namespace con_RIG
