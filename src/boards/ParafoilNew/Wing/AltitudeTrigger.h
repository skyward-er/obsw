/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <ParafoilNew/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Singleton.h>

namespace Payload
{

class AltitudeTrigger : public Boardcore::Singleton<AltitudeTrigger>
{
    friend class Boardcore::Singleton<AltitudeTrigger>;

public:
    // Update method that posts a FLIGHT_WING_ALT_REACHED when the correct
    // altitude is reached
    void update();

    // Method to set the altitude where trigger the dpl event
    void setDeploymentAltitude(float altitude);

private:
    AltitudeTrigger();

    // The altitude could be different from the default one
    float altitude;

    float fallingAltitude;

    // Number of times that the algorithm detects to be below the fixed
    // altitude
    int confidence;
};

}  // namespace Payload
