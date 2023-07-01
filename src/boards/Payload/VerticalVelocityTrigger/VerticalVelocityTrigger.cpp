/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Raul Radu
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

#include <Payload/BoardScheduler.h>
#include <Payload/Configs/FailSafeConfig.h>
#include <Payload/NASControllerMock/NASControllerMock.h>
#include <Payload/VerticalVelocityTrigger/VerticalVelocityTrigger.h>
#include <common/Events.h>
#include <events/EventBroker.h>

#include <functional>

using namespace std;
using namespace Boardcore;
using namespace Common;

namespace Payload
{

VerticalVelocityTrigger::VerticalVelocityTrigger()
{
    confidence = 0;
    running    = false;
}

bool VerticalVelocityTrigger::startModule()
{
    return ModuleManager::getInstance()
        .get<BoardScheduler>()
        ->getScheduler(miosix::MAIN_PRIORITY)
        ->addTask(bind(&VerticalVelocityTrigger::update, this),
                  FailSafe::FAILSAFE_VERTICAL_VELOCITY_TRIGGER_PERIOD);
}

void VerticalVelocityTrigger::enable()
{
    confidence = 0;
    running    = true;
}

void VerticalVelocityTrigger::disable() { running = false; }

bool VerticalVelocityTrigger::isActive() { return running; }

void VerticalVelocityTrigger::update()
{
    if (running)
    {
        float verticalVelocity = -ModuleManager::getInstance()
                                      .get<NASController>()
                                      ->getNasState()
                                      .vd;
        if (verticalVelocity < FailSafe::FAILSAFE_VERTICAL_VELOCITY_THRESHOLD)
        {
            confidence++;
        }
        else
        {
            confidence = 0;
        }
        if (confidence >=
            FailSafe::FAILSAFE_VERTICAL_VELOCITY_TRIGGER_CONFIDENCE)
        {
            confidence = 0;
            EventBroker::getInstance().post(FLIGHT_FAILSAFE_TRIGGERED,
                                            TOPIC_FLIGHT);
            running = false;
        }
    }
}
}  // namespace Payload
