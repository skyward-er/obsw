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

#include <Singleton.h>
#include <diagnostic/PrintLogger.h>
#include <events/FSM.h>

#include "AirBrakesControllerData.h"

#ifndef INTERP
#include <algorithms/AirBrakes/AirBrakesPI.h>
#else
#include <algorithms/AirBrakes/AirBrakesInterp.h>
#endif

namespace Main
{

class AirBrakesController : public Boardcore::FSM<AirBrakesController>,
                            public Boardcore::Singleton<AirBrakesController>
{
    friend Boardcore::Singleton<AirBrakesController>;

public:
    bool start() override;

    void update();

    AirBrakesControllerStatus getStatus();

    void state_init(const Boardcore::Event& event);
    void state_idle(const Boardcore::Event& event);
    void state_shadow_mode(const Boardcore::Event& event);
    void state_active(const Boardcore::Event& event);
    void state_end(const Boardcore::Event& event);

private:
    AirBrakesController();
    ~AirBrakesController();

    AirBrakesControllerStatus status;

    void logStatus(AirBrakesControllerState state);

    void wiggleServo();

#ifndef INTERP
    Boardcore::AirBrakesPI abk;
#else
    Boardcore::AirBrakesInterp abk;
#endif
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("main.abk");
};

}  // namespace Main
