/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Federico Lolli
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

#include <stdint.h>

#include <iostream>
#include <string>

namespace Antennas
{

enum class SMControllerState : uint8_t
{
    INIT = 0,
    INIT_ERROR,
    INIT_DONE,
    INSERT_INFO,
    ARMED,
    ARMED_NF,
    TEST,
    TEST_NF,
    CALIBRATE,
    FIX_ANTENNAS,
    FIX_ROCKET,
    FIX_ROCKET_NF,
    ACTIVE,
    ACTIVE_NF,
    /**
     * @brief macro state for configuration (init, init_error,
     * init_done, state_insert_info)
     */
    CONFIG,
    /**
     * @brief macro state for feedback (armed, test, calibrate,
     * fix_antennas, fix_rocket, active)
     */
    FEEDBACK,
    /**
     * @brief macro state for no feedback (armed_nf, test_nf,
     * fix_rocket_nf, active_nf)
     */
    NO_FEEDBACK
};

struct SMControllerStatus
{
    uint64_t timestamp;
    SMControllerState state;

    static std::string header() { return "timestamp,state\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)state << "\n";
    }
};

}  // namespace Antennas
