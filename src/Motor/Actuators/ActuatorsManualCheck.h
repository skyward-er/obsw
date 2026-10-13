/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Leonardo Montecchi
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

#include <Motor/Configs/ActuatorsConfig.h>
#include <common/MavlinkHydra.h>

namespace Motor
{
namespace ActuatorsManualCheck
{

constexpr bool is_manual(ServosList x)
{
    for (size_t i = 0; i < Config::Servos::MANUAL_VALVES.size(); i++)
        if (Config::Servos::MANUAL_VALVES[i] == x)
            return true;
    return false;
}

struct ManualValve
{
    ServosList value;

    consteval ManualValve(ServosList&& x) : value(x)
    {
        if (!is_manual(x))
            throw "Tried to call a manual valve method on a non-manual valve";
    }

    consteval ManualValve(const ServosList& x) : value(x)
    {
        if (!is_manual(x))
            throw "Tried to call a manual valve method on a non-manual valve";
    }

    ManualValve(ServosList& x) : value(x) {}
};

}  // namespace ActuatorsManualCheck
}  // namespace Motor
