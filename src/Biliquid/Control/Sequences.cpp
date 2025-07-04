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

#include <Biliquid/Actuators/Actuators.h>

#include <chrono>

#include "Sequence.h"
#include "SequenceManager.h"

using namespace std::chrono;

namespace Biliquid
{

void Sequence1::activate()
{
    actuators.openValve(Valve::MAIN_OX, 0.20f);
    actuators.openValve(Valve::MAIN_FUEL, 0.20f);
}

void Sequence1::deactivate() { actuators.closeAll(); }

void Sequence2::activate()
{
    // Disable sequence 1 when this sequence is active
    manager.disable(ControlSequence::SEQUENCE_1);

    constexpr auto DT = 1s;

    // TODO: animate valve opening based on time
    actuators.openValve(Valve::MAIN_OX, 1.0f);
    actuators.openValve(Valve::MAIN_FUEL, 1.0f);
}

void Sequence2::deactivate()
{
    actuators.closeAll();
    manager.enable(ControlSequence::SEQUENCE_1);
}

void Sequence3::activate()
{
    constexpr int N   = 5;
    constexpr auto DT = 5s;

    for (int i = 0; i < N; i++)
    {
        float position = i * (1.0f / N);
        actuators.openValve(Valve::MAIN_OX, position);
        actuators.openValve(Valve::MAIN_FUEL, position);

        miosix::Thread::nanoSleep(duration_cast<nanoseconds>(DT).count());
    }

    // Close the valves after the sequence
    actuators.closeValve(Valve::MAIN_OX);
    actuators.closeValve(Valve::MAIN_FUEL);
}

void Sequence3::deactivate() { actuators.closeAll(); }

}  // namespace Biliquid
