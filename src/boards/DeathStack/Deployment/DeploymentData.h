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

#include <drivers/hbridge/HBridgeData.h>
#include <stdint.h>

#include <iostream>
#include <string>

namespace DeathStackBoard
{

/**
 * @brief Enum defining the possibile FSM states.
 */
enum class DeploymentControllerState : uint8_t
{
    INITIALIZATION = 0,
    IDLE,
    NOSECONE_EJECTION,
    CUTTING,
    TEST_CUTTING_PRIMARY,
    TEST_CUTTING_BACKUP,
};

/**
 * @brief Structure defining the overall controller status.
 */
struct DeploymentStatus
{
    uint64_t timestamp;
    DeploymentControllerState state = DeploymentControllerState::INITIALIZATION;
    bool cutters_enabled            = false;
    float servo_position            = 0.0f;

    static std::string header()
    {
        return "timestamp,state,cutters_enabled,servo_position\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)state << "," << (int)cutters_enabled
           << "," << servo_position << "\n";
    }
};

}  // namespace DeathStackBoard
