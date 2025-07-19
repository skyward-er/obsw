/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include "SequenceManager.h"

#include <Biliquid/Control/Events.h>
#include <Biliquid/Debug.h>
#include <Biliquid/hwmapping.h>
#include <events/EventBroker.h>
#include <fmt/format.h>

using namespace std::chrono;
using namespace Boardcore;
using namespace Biliquid;

namespace Biliquid
{
SequenceManager::SequenceManager(Actuators& actuators)
    : ActiveObject(miosix::STACK_DEFAULT_FOR_PTHREAD, miosix::PRIORITY_MAX - 2),
      actuators(actuators)
{
    EventBroker::getInstance().subscribe(this, Topics::CONTROL_SEQUENCE);
}

SequenceManager::~SequenceManager() { stop(); }

void SequenceManager::postEvent(const Event& ev) { eventQueue.put(ev); }

void SequenceManager::IRQpostEvent(Event ev) { eventQueue.IRQput(ev); }

void SequenceManager::run()
{
    // Save the current context to return to it when a wait is interrupted
    if (setjmp(eventLoop) > 0)
    {
        if (waitEvent != -1)
        {
            EventBroker::getInstance().removeDelayed(waitEvent);
            waitEvent = -1;
        }
    }

    while (!shouldStop())
    {
        eventQueue.waitUntilNotEmpty();
        auto e = eventQueue.pop();
        handleEvent(e);
    }
}

void SequenceManager::waitFor(Boardcore::Event ev,
                              std::chrono::nanoseconds duration)
{
    // Post the continue event at the given time, simulating a wait
    waitEvent = EventBroker::getInstance().postDelayed(
        ev, Topics::CONTROL_SEQUENCE,
        duration_cast<milliseconds>(duration).count());

    // Keep handling events until the continue event is received
    while (!shouldStop())
    {
        eventQueue.waitUntilNotEmpty();
        auto e = eventQueue.pop();

        // Return to the waiter if the continue event is received
        if (e == ev)
        {
            PRINT_DEBUG("Returning from wait after {}\n", eventToString(ev));
            return;
        }

        handleEvent(e);
    }
}

void SequenceManager::handleEvent(Event ev)
{
    PRINT_DEBUG("Handling event: {}\n", eventToString(ev));

#define CASE_SEQUENCE(seq)                                                   \
    case Events::START_SEQUENCE_##seq:                                       \
    {                                                                        \
        activeSequence = ControlSequence::SEQUENCE_##seq;                    \
        auto context =                                                       \
            SequenceContext{*this, Events::CONTINUE_SEQUENCE_##seq};         \
        Sequence##seq::start(context, actuators);                            \
        hwmapping::ActionLed::low();                                         \
        /* Return to main event loop to cancel waits from other sequences */ \
        longjmp(eventLoop, 1);                                               \
    }                                                                        \
    break;                                                                   \
    case Events::STOP_SEQUENCE_##seq:                                        \
    {                                                                        \
        if (activeSequence.load() != ControlSequence::SEQUENCE_##seq)        \
        {                                                                    \
            PRINT_DEBUG("\tIgnoring STOP event for inactive sequence " #seq  \
                        "\n");                                               \
            return;                                                          \
        }                                                                    \
        auto context =                                                       \
            SequenceContext{*this, Events::CONTINUE_SEQUENCE_##seq};         \
        Sequence##seq::stop(context, actuators);                             \
        activeSequence = ControlSequence::NONE;                              \
        hwmapping::ActionLed::low();                                         \
        /* Return to main event loop to cancel waits from other sequences */ \
        longjmp(eventLoop, 1);                                               \
    }                                                                        \
    break

    switch (ev)
    {
        CASE_SEQUENCE(1);
        CASE_SEQUENCE(2);
        CASE_SEQUENCE(3);

        case Events::CONTINUE_SEQUENCE_1:
        case Events::CONTINUE_SEQUENCE_2:
        case Events::CONTINUE_SEQUENCE_3:
            PRINT_DEBUG("\tIgnoring CONTINUE event for inactive sequence\n");
            hwmapping::ActionLed::low();
            break;

        default:
            fmt::print("\t*** Unhandled event: {}\n", eventToString(ev));
            break;
    }
}
}  // namespace Biliquid
