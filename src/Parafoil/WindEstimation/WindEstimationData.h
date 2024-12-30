/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Basso
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

#include <units/Speed.h>

#include <Eigen/Core>
#include <iostream>
#include <string>

namespace Parafoil
{

/**
 * Wind Estimation Scheme data.
 */
struct WindEstimationData
{
    uint64_t timestamp = 0;
    Boardcore::Units::Speed::MeterPerSecond velocityNorth;
    Boardcore::Units::Speed::MeterPerSecond velocityEast;
    bool calibration = false;  ///< True if the wind estimation is in
                               ///< calibration mode, false otherwise

    static std::string header()
    {
        return "timestamp,velocityNorth,velocityEast,calibration\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << velocityNorth << "," << velocityEast << ","
           << calibration << "\n ";
    }
};

struct GeoVelocity
{
    Boardcore::Units::Speed::MeterPerSecond vn{0};  // North velocity
    Boardcore::Units::Speed::MeterPerSecond ve{0};  // East velocity

    /**
     * @brief Calculate the squared norm of the velocity vector.
     */
    float normSquared() const
    {
        return vn.value() * vn.value() + ve.value() * ve.value();
    }

    Eigen::Vector2f asVector() const { return {vn.value(), ve.value()}; }
};

}  // namespace Parafoil
