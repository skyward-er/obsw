/* Copyright (c) 2023-2024 Skyward Experimental Rocketry
 * Authors: Federico Mandelli, Nicclò Betto
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

#include "AltitudeTrigger.h"

#include <Payload/BoardScheduler.h>
#include <Payload/Configs/WingConfig.h>
#include <Payload/StateMachines/NASController/NASController.h>
#include <common/Events.h>
#include <events/EventBroker.h>

using namespace Boardcore;
using namespace Common;
namespace config = Payload::Config;

namespace Payload
{

bool AltitudeTrigger::start()
{
    auto& scheduler = getModule<BoardScheduler>()->altitudeTrigger();

    auto task = scheduler.addTask([this] { update(); },
                                  config::AltitudeTrigger::UPDATE_RATE);

    if (task == 0)
        return false;

    started = true;
    return true;
}

bool AltitudeTrigger::isStarted() { return started; }

void AltitudeTrigger::enable()
{
    if (running)
        return;

    confidence = 0;
    running    = true;
}

void AltitudeTrigger::disable() { running = false; }

bool AltitudeTrigger::isEnabled() { return running; }

void AltitudeTrigger::setDeploymentAltitude(float altitude)
{
    targetAltitude = altitude;
}

void AltitudeTrigger::update()
{
    if (!running)
        return;

    // NED frame, flip the D sign to get above-ground-level altitude
    auto nasState  = getModule<NASController>()->getNasState();
    float altitude = -nasState.d;

    if (altitude < targetAltitude)
        confidence++;
    else
        confidence = 0;

    if (confidence >= config::AltitudeTrigger::CONFIDENCE)
    {
        confidence = 0;
        EventBroker::getInstance().post(ALTITUDE_TRIGGER_ALTITUDE_REACHED,
                                        TOPIC_ALT);
        running = false;
    }
}

}  // namespace Payload
