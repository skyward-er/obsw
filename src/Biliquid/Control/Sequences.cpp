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

#include "SequenceManager.h"

using namespace std::chrono;

namespace Biliquid
{

namespace Sequence1
{
void start(SequenceManager& manager, Actuators& actuators)
{
    actuators.openValve(Valve::MAIN_OX, 0.20f);
    actuators.openValve(Valve::MAIN_FUEL, 0.20f);
}

void stop(SequenceManager& manager, Actuators& actuators)
{
    actuators.closeAll();
}
}  // namespace Sequence1

namespace Sequence2
{
void start(SequenceManager& manager, Actuators& actuators)
{
    constexpr auto DT = 1s;

    // TODO: animate valve opening based on time
    actuators.openValve(Valve::MAIN_OX, 1.0f);
    actuators.openValve(Valve::MAIN_FUEL, 1.0f);
}

void stop(SequenceManager& manager, Actuators& actuators)
{
    actuators.closeAll();
}
}  // namespace Sequence2

namespace Sequence3
{
void start(SequenceManager& manager, Actuators& actuators)
{
    constexpr int N   = 5;
    constexpr auto DT = 5s;

    for (int i = 0; i < N; i++)
    {
        float position = i * (1.0f / N);
        actuators.openValve(Valve::MAIN_OX, position);
        actuators.openValve(Valve::MAIN_FUEL, position);

        manager.waitFor(DT);
    }

    // Close the valves after the sequence
    actuators.closeValve(Valve::MAIN_OX);
    actuators.closeValve(Valve::MAIN_FUEL);
}

void stop(SequenceManager& manager, Actuators& actuators)
{
    actuators.closeAll();
}
}  // namespace Sequence3
}  // namespace Biliquid
