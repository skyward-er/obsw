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

/**
 * This class is used to keep track of various main class
 * initialization errors.
 */

#pragma once

#include <string>
#include <ostream>

namespace ParafoilTestDev
{
    enum ParafoilTestComponentStatus
    {
        ERROR   = 0,
        OK      = 1
    };

    struct ParafoilTestStatus
    {
        //If there is an error, this uint8_t reports it(OR)
        uint8_t parafoil_test = OK;

        //Specific errors
        uint8_t logger          = OK;
        uint8_t eventBroker     = OK;
        uint8_t sensors         = OK;
        uint8_t FMM             = OK;
        uint8_t radio           = OK;

        /**
         * @brief Method to set a specific component in an error state
         */
        void setError(uint8_t ParafoilTestStatus::*component)
        {
            //Put the passed component to error state
            this->*component  = ERROR;
            //Logic OR
            parafoil_test       = ERROR;
        }


        static std::string header()
        {
            return "logger, eventBorker, sensors, radio\n";
        }

        void print(std::ostream& os)
        {
            os << (int)logger << "," << (int)eventBroker << "," << (int)sensors << "," << (int)radio << "\n";
        }
    };
}