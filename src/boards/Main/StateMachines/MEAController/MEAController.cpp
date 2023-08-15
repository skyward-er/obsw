/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <Main/Configs/MEAConfig.h>
#include <Main/StateMachines/MEAController/MEAController.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <events/EventBroker.h>

#include <functional>

using namespace Boardcore;
using namespace Common;
using namespace std;

namespace Main
{
MEAController::MEAController(TaskScheduler* sched)
    : FSM(&MEAController::state_idle), mea(getMEAKalmanConfig()),
      scheduler(sched)
{
    // Subscribe the class of the topics
    EventBroker::getInstance().subscribe(this, TOPIC_MOTOR);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

bool MEAController::start()
{
    size_t result = scheduler->addTask(bind(&MEAController::update, this),
                                       MEAConfig::UPDATE_PERIOD,
                                       TaskScheduler::Policy::RECOVER);
    return ActiveObject::start() && result != 0;
}

void MEAController::update() {}

MEAControllerStatus MEAController::getStatus()
{
    // Need to pause the kernel
    miosix::PauseKernelLock l;
    return status;
}

MEAState MEAController::getMEAState()
{
    // Need to pause the kernel
    miosix::PauseKernelLock l;
    return mea.getState();
}

void MEAController::state_idle(const Event& event) {}

void MEAController::state_ready(const Event& event) {}

void MEAController::state_armed(const Event& event) {}

void MEAController::state_shadow_mode(const Event& event) {}

void MEAController::state_active(const Event& event) {}

void MEAController::state_end(const Event& event) {}

void MEAController::logStatus(MEAControllerState state) {}

MEA::KalmanFilter::KalmanConfig MEAController::getMEAKalmanConfig() {}

}  // namespace Main