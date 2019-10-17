/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include "Motor/MotorData.h"
#include "ThermalCutter/CutterData.h"

namespace DeathStackBoard
{

enum DeploymentCTRLState : uint8_t
{
    DPL_IDLE = 0,
    CUTTING_PRIMARY,
    CUTTING_BACKUP,
    EJECTING_NC,
    MOVING_SERVO,
    RESETTING_SERVO,
    TESTING_PRIMARY,
    TESTING_BACKUP
};

struct DeploymentStatus
{
    long long timestamp;
    DeploymentCTRLState state;
    CutterStatus cutter_status;
    float servo_position;

    static std::string header()
    {
        return "timestamp,state,cutter_state,servo_position\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)state << "," << (int)cutter_status.state
           << "," << servo_position << "\n";
    }
};

}  // namespace DeathStackBoard