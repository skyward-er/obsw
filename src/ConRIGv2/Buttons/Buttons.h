/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Ettore Pane
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

#include <ConRIGv2/BoardScheduler.h>
#include <common/MavlinkOrion.h>
#include <diagnostic/PrintLogger.h>
#include <scheduler/TaskScheduler.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace ConRIGv2
{

class Radio;

class Buttons : public Boardcore::InjectableWithDeps<BoardScheduler, Radio>
{
public:
    Buttons();

    [[nodiscard]] bool start();

    mavlink_conrig_state_tc_t getState();

    void enableIgnition();
    void disableIgnition();

private:
    void resetState();

    void periodicStatusCheck();

    mavlink_conrig_state_tc_t state;

    // Counter guard to avoid spurious triggers
    uint8_t guard = 0;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("buttons");
};

}  // namespace ConRIGv2
