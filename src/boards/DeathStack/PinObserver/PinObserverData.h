/*
 * Copyright (c) 2019 Skyward Experimental Rocketry
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

#include <cstdint>

namespace DeathStackBoard
{

enum class ObservedPin : uint8_t
{
    LAUNCH   = 0,
    NOSECONE = 1
};

/**
 * @brief Struct represeting the status of an observed pin
 *
 */
struct PinStatus
{
    ObservedPin pin;

    long long last_state_change; //Last time the pin changed state
    uint8_t state; //Current state of the pin
    unsigned int num_state_changes = 0; 

    long long last_detection_time; //When a transition is detected

    PinStatus(ObservedPin pin) : pin(pin) {}
     static std::string header()
    {
        return "pin,last_state_change,state,num_state_changes,last_detection_time\n";
    }

    void print(std::ostream& os) const
    {
        os << (int)pin << "," << last_state_change << "," << (int)state << ","
           << num_state_changes << "," << last_detection_time << "\n";
    }
};

}  // namespace DeathStackBoard
