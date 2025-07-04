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

#include <Biliquid/hwmapping.h>
#include <drivers/interrupt/external_interrupts.h>
#include <kernel/scheduler/scheduler.h>

using namespace Biliquid;

SequenceManager* manager = nullptr;

IRQ_HANDLER DEWESOFT_INTERRUPT_1()
{
    if (!manager)
        return;

    hwmapping::IrqLed::high();

    int state = hwmapping::DewesoftInterrupt1::value();
    manager->IRQsetPending(ControlSequence::SEQUENCE_1, state);
}

IRQ_HANDLER DEWESOFT_INTERRUPT_2()
{
    if (!manager)
        return;

    hwmapping::IrqLed::high();

    int state = hwmapping::DewesoftInterrupt2::value();
    manager->IRQsetPending(ControlSequence::SEQUENCE_2, state);
}

IRQ_HANDLER DEWESOFT_INTERRUPT_3()
{
    if (!manager)
        return;

    hwmapping::IrqLed::high();

    int state = hwmapping::DewesoftInterrupt3::value();
    manager->IRQsetPending(ControlSequence::SEQUENCE_3, state);
}

namespace Biliquid
{
SequenceManager::SequenceManager(Actuators& actuators)
    : actuators(actuators), sequences({new Sequence1(actuators, *this),
                                       new Sequence2(actuators, *this),
                                       new Sequence3(actuators, *this)}),
      semaphore(0)
{
    thread = miosix::Thread::create(
        [](void* instance)
        { static_cast<SequenceManager*>(instance)->handlerThread(); },
        miosix::STACK_DEFAULT_FOR_PTHREAD, miosix::PRIORITY_MAX - 2, this,
        miosix::Thread::DEFAULT);

    manager = this;

    // Initialize interrupts
    enableExternalInterrupt(hwmapping::DewesoftInterrupt1::getPin(),
                            InterruptTrigger::RISING_FALLING_EDGE);
    enableExternalInterrupt(hwmapping::DewesoftInterrupt2::getPin(),
                            InterruptTrigger::RISING_FALLING_EDGE);
    enableExternalInterrupt(hwmapping::DewesoftInterrupt3::getPin(),
                            InterruptTrigger::RISING_FALLING_EDGE);
}

SequenceManager::~SequenceManager() { manager = nullptr; }

void SequenceManager::IRQsetPending(ControlSequence sequence, bool state)
{
    getSequence(sequence)->setPending(state);
    semaphore.IRQsignal();
}

void SequenceManager::handlerThread()
{
    while (true)
    {
        semaphore.wait();  // Wait for a signal to run sequences

        // Run any sequence that is pending
        for (auto sequence : sequences)
        {
            // Ignore non-pending sequences
            if (!sequence->pending)
                continue;

            sequence->pending = false;  // Reset pending state
            hwmapping::IrqLed::low();

            // Skip masked sequences
            if (sequence->masked)
                continue;

            if (sequence->state)
                sequence->activate();
            else
                sequence->deactivate();
        }
    }
}
}  // namespace Biliquid
