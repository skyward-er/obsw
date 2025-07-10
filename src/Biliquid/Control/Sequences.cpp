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
    actuators.openValve(Valve::MAIN_OX, 1.0f);
    actuators.openValve(Valve::MAIN_FUEL, 1.0f);

    manager.waitFor(1s);
    actuators.closeAll();
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
    actuators.openValve(Valve::MAIN_FUEL, 1.0f);
    manager.waitFor(100ms);
    actuators.openValve(Valve::MAIN_OX, 1.0f);
    manager.waitFor(900ms);

    actuators.closeAll();
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
    constexpr float Positions[] = {0.09f,  0.154f, 0.193f, 0.252f, 0.3f,
                                   0.345f, 0.39f,  0.437f, 0.484f};
    constexpr auto DT           = 2s;

    for (auto pos : Positions)
    {
        actuators.openValve(Valve::MAIN_OX, pos);
        actuators.openValve(Valve::MAIN_FUEL, pos);

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
