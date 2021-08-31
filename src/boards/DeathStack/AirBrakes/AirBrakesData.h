/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

namespace DeathStackBoard
{

/**
 * Enum defining the possibile FSM states.
 */
enum class AirBrakesControllerState : uint8_t
{
    INITIALIZATION = 0,
    IDLE,
    TEST_AEROBRAKES,
    SHADOW_MODE,
    ENABLED,
    DISABLED,
    END,
};

/**
 * Structure defining the output of the control algorithm.
 */
struct AirBrakesData
{
    uint64_t timestamp;
    float servo_position;
    float estimated_cd;
    float pid_error;
    float setpoint_z;
    float setpoint_vz;

    static std::string header()
    {
        return "timestamp,servo_position,estimated_cd,pid_error,setpoint_z,setpoint_vz\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << servo_position << "," << estimated_cd << ","
           << pid_error << "," << setpoint_z << "," << setpoint_vz << "\n";
    }
};

/**
 * Structure defining the overall controller status.
 */
struct AirBrakesControllerStatus
{
    uint64_t timestamp;
    AirBrakesControllerState state;

    static std::string header() { return "timestamp,state\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)state << "\n";
    }
};

/**
 * Structure to log the data relative to the airbrakes algorithm, including the
 * chosen trajectory (when the airbrakes are enabled for the first time).
 */
struct AirBrakesAlgorithmData
{
    uint64_t timestamp;
    float z;
    float vz;
    float vMod;

    static std::string header() { return "timestamp,z,vz,vMod\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << z << "," << vz << "," << vMod << "\n";
    }
};

struct AirBrakesChosenTrajectory
{
    uint8_t trajectory;

    static std::string header() { return "trajectory\n"; }

    void print(std::ostream& os) const { os << trajectory << "\n"; }
};

}  // namespace DeathStackBoard
