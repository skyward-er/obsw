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

enum class SMAState : uint8_t
{
    INIT = 0,       ///< 0
    INIT_ERROR,     ///< 1
    INIT_DONE,      ///< 2
    INSERT_INFO,    ///< 3
    ARMED,          ///< 4
    ARMED_NF,       ///< 5
    TEST,           ///< 6
    TEST_NF,        ///< 7
    CALIBRATE,      ///< 8
    FIX_ANTENNAS,   ///< 9
    FIX_ROCKET,     ///< 10
    FIX_ROCKET_NF,  ///< 11
    ARM_READY,      ///< 12
    ACTIVE,         ///< 13
    ACTIVE_NF,      ///< 14
    /**
     * @brief macro state for configuration (init, init_error,
     * init_done, state_insert_info)
     */
    CONFIG,  ///< 15
    /**
     * @brief macro state for feedback (armed, test, calibrate,
     * fix_antennas, fix_rocket, active)
     */
    FEEDBACK,  ///< 16
    /**
     * @brief macro state for no feedback (armed_nf, test_nf,
     * fix_rocket_nf, active_nf)
     */
    NO_FEEDBACK,  ///< 17
    INVALID,      ///< 18
};

struct SMAStatus
{
    uint64_t timestamp = 0;
    SMAState state     = SMAState::INVALID;

    static std::string header() { return "timestamp,state\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)state << "\n";
    }
};

}  // namespace Antennas
