/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Riccardo Musso, Emilio Corigliano
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

#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>

#include <Groundstation/Base/Buses.h>
#include <Groundstation/Base/Hub.h>
#include <Groundstation/Base/Radio/Radio.h>
#include <Groundstation/Base/Radio/RadioStatus.h>
#include <Groundstation/Common/Ports/Serial.h>
#include "Converter.h"

using namespace miosix;
using namespace Boardcore;

inline float randf() { return (std::rand() % 200 - 100) / 100.f; }

int main()
{
    constexpr int N = 10000;

    printf("Starting test\n");
    uint64_t start = TimestampTimer::getTimestamp();

    for (int i = 0; i < N; i++)
    {
        NEDCoords coords     = {randf(), randf(), randf()};
        AntennaAngles angles = rocketPositionToAntennaAngles(coords);
        printf("NED: %.2f ; %.2f ; %.2f -> Angles %.2f ; %.2f\n", coords.n,
               coords.e, coords.d, angles.theta1, angles.theta2);
    }

    uint64_t end = TimestampTimer::getTimestamp();
    printf("Took %llu millis for %d calls.\n", (end - start) / 1000ull, N);

    return 0;
}