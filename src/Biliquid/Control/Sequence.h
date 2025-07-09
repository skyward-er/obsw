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

#include <events/Event.h>

#include <chrono>

namespace Biliquid
{
class SequenceManager;
class Actuators;

enum class ControlSequence
{
    SEQUENCE_1 = 0,
    SEQUENCE_2,
    SEQUENCE_3,
    NONE,  //!< No sequence is active
};

namespace Sequence1
{
void start(SequenceManager& manager, Actuators& actuators);
void stop(SequenceManager& manager, Actuators& actuators);
}  // namespace Sequence1

namespace Sequence2
{
void start(SequenceManager& manager, Actuators& actuators);
void stop(SequenceManager& manager, Actuators& actuators);
}  // namespace Sequence2

namespace Sequence3
{
void start(SequenceManager& manager, Actuators& actuators);
void stop(SequenceManager& manager, Actuators& actuators);
}  // namespace Sequence3

}  // namespace Biliquid
