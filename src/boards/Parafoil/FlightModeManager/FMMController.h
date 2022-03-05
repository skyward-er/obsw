/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro
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

#include "FMMData.h"
#include <events/FSM.h>
#include <diagnostic/PrintLogger.h>

using namespace Boardcore;

namespace ParafoilTestDev
{
    /**
     * @brief FMM state machine
     */
    class FMMController : public FSM<FMMController>
    {
    public:
        FMMController();
        ~FMMController();

        void state_on_ground(const Event& ev);
        void state_flying(const Event& ev);
        void state_debug(const Event& ev);

    private:
        FMMControllerStatus status;
        PrintLogger logger = Logging::getLogger("FMM");
        Logger* SDlogger = &Logger::getInstance();

        /* --- LOGGER --- */

        void logStatus(FMMControllerState state);
        void logStatus();
    };
}  // namespace ParafoilTestDev
