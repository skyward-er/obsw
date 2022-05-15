/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <diagnostic/PrintLogger.h>
#include <events/FSM.h>

#include "AirBrakesData.h"

namespace Main
{

class AirBrakesController : public Boardcore::FSM<AirBrakesController>
{
public:
    AirBrakesController();
    ~AirBrakesController();

    void state_init(const Boardcore::Event& ev);
    void state_idle(const Boardcore::Event& ev);
    void state_shadow_mode(const Boardcore::Event& ev);
    void state_active(const Boardcore::Event& ev);
    void state_end(const Boardcore::Event& ev);

private:
    AirBrakesControllerStatus status;

    void wiggle_servo();

    void logStatus(AirBrakesControllerState state);

    uint16_t shadow_mode_timeout_event_id = 0;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("main.abk");
};

}  // namespace Main
