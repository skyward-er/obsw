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

#include <events/EventHandler.h>
#include <utils/collections/IRQCircularBuffer.h>

#include <array>
#include <atomic>
#include <chrono>
#include <csetjmp>

#include "Sequence.h"

namespace Biliquid
{
class Actuators;

class SequenceManager : public Boardcore::EventHandlerBase,
                        public Boardcore::ActiveObject
{
public:
    explicit SequenceManager(Actuators& actuators);
    virtual ~SequenceManager() noexcept;

    void postEvent(const Boardcore::Event& ev) override;
    void IRQpostEvent(Boardcore::Event ev);

    void run() override;

    /**
     * @brief Waits asynchronously for the given duration.
     *
     * @note Waits CANNOT be nested, intermediate waits will be lost!
     */
    void waitFor(std::chrono::nanoseconds duration);

    void handleEvent(Boardcore::Event ev);

private:
    Actuators& actuators;

    std::atomic<ControlSequence> activeSequence = {ControlSequence::NONE};

    Boardcore::IRQCircularBuffer<Boardcore::Event, 32>
        eventQueue;  ///< Event queue for the sequences

    std::jmp_buf eventLoop;
    int32_t waitEvent = -1;
};
}  // namespace Biliquid
