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

#pragma once

#include <Biliquid/hwmapping.h>
#include <miosix.h>

#include <array>

#include "Sequence.h"

namespace Biliquid
{
class Actuators;

class SequenceManager
{
public:
    SequenceManager(Actuators& actuators);
    ~SequenceManager();

    void IRQsetPending(ControlSequence sequence, bool state);

    Sequence* getSequence(ControlSequence sequence)
    {
        return sequences[static_cast<size_t>(sequence)];
    }

    void enable(ControlSequence sequence)
    {
        getSequence(sequence)->masked = false;
    }

    void disable(ControlSequence sequence)
    {
        getSequence(sequence)->masked = true;
    }

private:
    void handlerThread();

    Actuators& actuators;
    std::array<Sequence*, 3> sequences;

    miosix::Thread* thread = nullptr;  ///< Sequence runner thread
    miosix::Semaphore semaphore;       ///< Semaphore for thread synchronization
};
}  // namespace Biliquid
