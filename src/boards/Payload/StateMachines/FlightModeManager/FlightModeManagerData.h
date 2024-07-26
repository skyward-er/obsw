/* Copyright (c) 2024 Skyward Experimental Rocketry
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

#include <cstdint>
#include <ostream>
#include <string>

namespace Payload
{

enum class FlightModeManagerState : uint8_t
{
    ON_GROUND = 0,
    ON_GROUND_INIT,
    ON_GROUND_INIT_ERROR,
    ON_GROUND_INIT_DONE,
    ON_GROUND_SENSOR_CALIBRATION,
    ON_GROUND_ALGORITHM_CALIBRATION,
    ON_GROUND_DISARMED,
    ON_GROUND_TEST_MODE,
    ARMED,
    FLYING_ASCENDING,
    FLYING_DROGUE_DESCENT,
    FLYING_WING_DESCENT,
    LANDED,
};

struct FlightModeManagerStatus
{
    uint64_t timestamp           = 0;
    FlightModeManagerState state = FlightModeManagerState::ON_GROUND;

    static std::string header() { return "timestamp,state\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)state << "\n";
    }
};

}  // namespace Payload
