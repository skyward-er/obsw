/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro, Federico Mandelli
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

#include <Parafoil/AltitudeTrigger/AltitudeTrigger.h>
#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Configs/WingConfig.h>
#include <Parafoil/StateMachines/NASController/NASController.h>
#include <common/events/Events.h>
#include <events/EventBroker.h>

#include <functional>
#include <utils/ModuleManager/ModuleManager.hpp>

using namespace std;
using namespace Parafoil::WingConfig;
using namespace Boardcore;
using namespace Common;

namespace Parafoil
{

AltitudeTrigger::AltitudeTrigger()
{
    ModuleManager::getInstance().get<BoardScheduler>()->getScheduler().addTask(
        bind(&AltitudeTrigger::update, this), WING_ALTITUDE_TRIGGER_PERIOD);
    confidence = 0;
    running    = false;
}

void AltitudeTrigger::enable()
{
#ifdef PRF_TEST
    startingAltitude = 0;
#else
    startingAltitude = -NASController::getInstance().getNasState().d;
#endif
    confidence = 0;
    running    = true;
}

void AltitudeTrigger::disable() { running = false; }

bool AltitudeTrigger::isActive() { return running; }

void AltitudeTrigger::update()
{
    if (running)
    {
#ifdef PRF_TEST
        float height     = 0;
        startingAltitude = startingAltitude + 2.5;
#else
        float height = -NASController::getInstance().getNasState().d;
#endif
        if (startingAltitude - height > WING_ALTITUDE_TRIGGER_FALL)
        {
            confidence++;
        }
        else
        {
            confidence = 0;
        }
        if (confidence >= WING_ALTITUDE_TRIGGER_CONFIDENCE)
        {
            confidence       = 0;
            startingAltitude = 0;
            EventBroker::getInstance().post(FLIGHT_WING_ALT_PASSED,
                                            TOPIC_ALGOS);
            running = false;
        }
    }
}
}  // namespace Parafoil
