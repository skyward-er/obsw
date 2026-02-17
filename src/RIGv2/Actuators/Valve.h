/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Riccardo Sironi
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

#include "ValveServo.h"
#include "ValveServoPCA.h"
#include "ValveSolenoid.h"

namespace RIGv2
{
class Valve
{
public:
    struct ValveConfig
    {
        float limit                 = 1.0;    ///< Movement range limit
        bool flipped                = false;  ///< Whether the servo is flipped
        uint32_t defaultOpeningTime = 1000;   // Default opening time [ms]
        float defaultMaxAperture    = 1.0;    // Max aperture

        uint8_t openingEvent = 0;  ///< Event to fire after opening
        uint8_t closingEvent = 0;  ///< Event to fire after closing
        uint32_t openingTimeRegKey =
            CONFIG_ID_DEFAULT_OPENING_TIME;  ///< Registry key for opening
                                             ///< time
        uint32_t maxApertureRegKey =
            CONFIG_ID_DEFAULT_MAX_APERTURE;  ///< Registry key for max
                                             ///< aperture
    };

    Valve(std::unique_ptr<ValveInterface> interface, const ValveConfig& config)
        : servo(std::move(interface)), config(config)
    {
    }
    ~Valve() {};

    // Move-only
    Valve(Valve&&)                 = default;
    Valve& operator=(Valve&&)      = default;
    Valve(const Valve&)            = delete;
    Valve& operator=(const Valve&) = delete;

    void unsafeSetServoPosition(float position);

    bool isServoOpen();

    float getServoPosition();

    const ValveConfig getConfig() const;

private:
    std::unique_ptr<ValveInterface> servo;
    ValveConfig config;
};
}  // namespace RIGv2
