/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Mandelli Federico
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

#include <drivers/canbus/Canbus.h>

namespace common
{

class CanID
{
public:
    enum BoardID
    {
        MAIN_FC      = 0,  // 00 higher priority
        PAYLOAD_FC   = 1,  // 01
        AUXILIARY_FC = 2   // 10
    };
    enum MessageID
    {                                 // ids of the messages max 8 (0xFF)
        LIFT_OFF             = 0X01,  // higher priority
        ARMED                = 0X02,
        PITOT_PACKET         = 0X03,
        AIRBREAKS_PERCENTAGE = 0X04  // etc, etc
    };

    CanID(BoardID b)
    {
        boardType = b;  // boardtype should be the 10th and 9th bits
        boardType << 8;
    }
    // return last setted id
    uint16_t getId() { return id; }

    void setId(MessageID msg, bool override)
    {
        if (override)
        {
            id = 0;
        }
        else
        {
            id = 1;
        }
        id << 10;  // 5 zeros||id||10 zeros
        id = id | boardType | msg;
    }

private:
    uint16_t id =
        2047;  // default value 2047 (eleven 1's) lowest possible priority
    BoardID boardType;
};
}  // namespace common
