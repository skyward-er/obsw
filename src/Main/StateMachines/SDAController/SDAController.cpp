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

#include "SDAController.h"

#include <Main/Configs/SDAConfig.h>
#include <Main/StateMachines/SDAController/SDAControllerData.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <events/EventBroker.h>

using namespace Main;
using namespace Boardcore;
using namespace miosix;
using namespace Common;

SDAController::SDAController()
    : FSM{&SDAController::state_init, miosix::STACK_DEFAULT_FOR_PTHREAD,
          Config::Scheduler::SDA_PRIORITY},
      sda{}
{
    // EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

bool SDAController::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->getSdaScheduler();

    sdaID =
        scheduler.addTask([this]() { update(); }, Config::SDA::UPDATE_RATE_SDA);

    if (sdaID == 0)
    {
        LOG_ERR(logger, "Failed to add SDA task");
        return false;
    }

    if (!FSM::start())
    {
        LOG_ERR(logger, "Failed to start SDA FSM");
        return false;
    }

    return true;
}
