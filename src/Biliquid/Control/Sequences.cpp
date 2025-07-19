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
void start(SequenceContext& context, Actuators& actuators)
{
    actuators.openValve(Valve::MAIN_FUEL, 0.19184f);
    context.waitFor(100ms);
    actuators.openValve(Valve::MAIN_OX, 0.21439f);

    context.waitFor(2500ms);
    actuators.closeAll();
}

void stop(SequenceContext& context, Actuators& actuators)
{
    actuators.closeAll();
}
}  // namespace Sequence1

namespace Sequence2
{
void start(SequenceContext& context, Actuators& actuators)
{
    constexpr auto DT = 1s;

    actuators.openAnimateValve(Valve::MAIN_OX, 1.0f, DT);
    actuators.openAnimateValve(Valve::MAIN_FUEL, 1.0f, DT);

    context.waitFor(DT + 1000ms);
    actuators.closeAll();
}

void stop(SequenceContext& context, Actuators& actuators)
{
    actuators.closeAll();
}
}  // namespace Sequence2

namespace Sequence3
{
void start(SequenceContext& context, Actuators& actuators)
{
    constexpr float Positions[] = {0.14388f, 0.16786f, 0.19184f,
                                   0.21439f, 0.23694f, 0.25883f};
    constexpr auto DT           = 2s;

    for (auto pos : Positions)
    {
        actuators.openValve(Valve::MAIN_OX, pos);
        actuators.openValve(Valve::MAIN_FUEL, pos);

        context.waitFor(DT);
    }

    // Close the valves after the sequence
    actuators.closeValve(Valve::MAIN_OX);
    actuators.closeValve(Valve::MAIN_FUEL);
}

void stop(SequenceContext& context, Actuators& actuators)
{
    actuators.closeAll();
}
}  // namespace Sequence3
}  // namespace Biliquid
