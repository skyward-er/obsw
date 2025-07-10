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
#include <drivers/interrupt/external_interrupts.h>
#include <events/EventBroker.h>
#include <fmt/format.h>

using Boardcore::Event;
using namespace Boardcore;
using namespace Biliquid;

static SequenceManager* manager = nullptr;

IRQ_HANDLER DEWESOFT_INTERRUPT_1()
{
    if (!manager)
        return;

    hwmapping::IrqLed::high();

    int state = hwmapping::DewesoftInterrupt1::value();
    if (state)
        manager->IRQpostEvent(Events::START_SEQUENCE_1);
    else
        manager->IRQpostEvent(Events::STOP_SEQUENCE_1);
}

IRQ_HANDLER DEWESOFT_INTERRUPT_2()
{
    if (!manager)
        return;

    hwmapping::IrqLed::high();

    int state = hwmapping::DewesoftInterrupt2::value();
    if (state)
        manager->IRQpostEvent(Events::START_SEQUENCE_2);
    else
        manager->IRQpostEvent(Events::STOP_SEQUENCE_2);
}

IRQ_HANDLER DEWESOFT_INTERRUPT_3()
{
    if (!manager)
        return;

    hwmapping::IrqLed::high();

    int state = hwmapping::DewesoftInterrupt3::value();
    if (state)
        manager->IRQpostEvent(Events::START_SEQUENCE_3);
    else
        manager->IRQpostEvent(Events::STOP_SEQUENCE_3);
}

namespace Biliquid
{
SequenceManager::SequenceManager(Actuators& actuators)
    : ActiveObject(miosix::STACK_DEFAULT_FOR_PTHREAD, miosix::PRIORITY_MAX - 2),
      actuators(actuators)
{
    manager = this;

    EventBroker::getInstance().subscribe(this, Topics::CONTROL_SEQUENCE);
}

SequenceManager::~SequenceManager()
{
    manager = nullptr;
    stop();
}

void SequenceManager::postEvent(const Event& ev) { eventQueue.put(ev); }

void SequenceManager::IRQpostEvent(Event ev) { eventQueue.IRQput(ev); }

void SequenceManager::run()
{
    setjmp(eventLoop);

    while (!shouldStop())
    {
        eventQueue.waitUntilNotEmpty();
        auto e = eventQueue.pop();
        handleEvent(e);
    }
}

void SequenceManager::waitFor(std::chrono::nanoseconds duration)
{
    Boardcore::EventBroker::getInstance().postDelayed(
        Events::CONTINUE_SEQUENCE, duration.count(), Topics::CONTROL_SEQUENCE);

    while (!shouldStop())
    {
        eventQueue.waitUntilNotEmpty();
        auto e = eventQueue.pop();

        // Return to the waiter context if the continue event is received
        if (e == Events::CONTINUE_SEQUENCE)
            return;

        handleEvent(e);
    }
}

void SequenceManager::handleEvent(Event ev)
{
    bool handled = true;

#define CASE_SEQUENCE(seq)                                            \
    case Events::START_SEQUENCE_##seq:                                \
        PRINT_DEBUG("START sequence " #seq "\n");                     \
        activeSequence = ControlSequence::SEQUENCE_##seq;             \
        Sequence##seq::start(*this, actuators);                       \
        break;                                                        \
    case Events::STOP_SEQUENCE_##seq:                                 \
        if (activeSequence.load() != ControlSequence::SEQUENCE_##seq) \
        {                                                             \
            PRINT_DEBUG("Sequence " #seq                              \
                        " is not active, ignoring STOP "              \
                        "event\n");                                   \
            return;                                                   \
        }                                                             \
        PRINT_DEBUG("STOP sequence " #seq "\n");                      \
        Sequence##seq::stop(*this, actuators);                        \
        activeSequence = ControlSequence::NONE;                       \
        /* Return to the main run loop context to exit any waits */   \
        longjmp(eventLoop, 1);                                        \
        break

    switch (ev)
    {
        CASE_SEQUENCE(1);
        CASE_SEQUENCE(2);
        CASE_SEQUENCE(3);

        default:
            fmt::print("*** Unhandled event: {}", (int)ev);
            handled = false;
            break;
    }

    if (handled)
        hwmapping::IrqLed::low();
}
}  // namespace Biliquid
