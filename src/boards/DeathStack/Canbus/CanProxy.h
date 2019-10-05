/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise De Faveri
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

#include <Common.h>
#include <drivers/canbus/CanManager.h>
#include <drivers/canbus/CanUtils.h>
#include <DeathStack/LoggerService/LoggerService.h>

namespace DeathStackBoard
{

/**
 * This class is interposed between the OBSW and the Canbus driver. 
 * Canbus initialization and status logging is done here.
 */
class CanProxy
{
public:
    CanProxy(CanManager* c);
    ~CanProxy() {};

    /*
     * Sending proxy function: sends and logs the status.
     *
     * @param id       Id of the message (aka topic)
     * @para message   message as byte array
     * @param len      length of the message (max 8 bytes, truncated if grater)
     * @return true    if the message was sent correctly
     */
    bool send(uint16_t id, const uint8_t* message, uint8_t len);


    /*
     * Getters
     */
    CanBus* getBus() { return bus; }
    CanStatus getStatus() { return bus->getStatus(); }

private:
    CanBus* bus;
};

} /* namespace DeathStackBoard */
