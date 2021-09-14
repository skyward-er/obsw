/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <cstdint>
#include <ostream>
#include <string>

namespace DeathStackBoard
{

enum DeathStackComponentStatus
{
    COMP_ERROR = 0,
    COMP_OK    = 1
};

struct DeathStackStatus
{
    // Logic OR of all components errors.
    uint8_t death_stack = COMP_OK;

    uint8_t logger         = COMP_OK;
    uint8_t ev_broker      = COMP_OK;
    uint8_t pin_obs        = COMP_OK;
    uint8_t sensors        = COMP_OK;
    uint8_t radio          = COMP_OK;
    uint8_t state_machines = COMP_OK;

    /**
     * @brief Helper method to signal an error in the DeathStackStatus struct.
     *
     * @param component_status Pointer to a member of DeathStackStatus
     * Eg: setError(&DeathStackStatus::dpl)
     */
    void setError(uint8_t DeathStackStatus::*component_status)
    {
        this->*component_status = COMP_ERROR;
        death_stack             = COMP_ERROR;
    }

    static std::string header()
    {
        return "logger,ev_broker,pin_obs,sensors,radio,state_machines\n";
    }

    void print(std::ostream& os)
    {
        os << (int)logger << "," << (int)ev_broker << "," << (int)pin_obs << ","
           << (int)sensors << "," << (int)radio << "," << (int)state_machines
           << "\n";
    }
};

}  // namespace DeathStackBoard