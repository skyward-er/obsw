/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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

#include <drivers/canbus/CanProtocol.h>

namespace common
{

class MockAirBrakes
{
public:
    // TODO: update to use pressure data
    uint8_t getData()
    {
        miosix::Lock<miosix::FastMutex> l(mutex);
        updated = false;
        return percentage;
    }

    bool isUpdated()
    {
        miosix::Lock<miosix::FastMutex> l(mutex);
        return updated;
    }

    void setData(Boardcore::Canbus::CanData packet)
    {
        miosix::Lock<miosix::FastMutex> l(mutex);
        percentage = packet.payload[0];
        updated    = true;
    }

    Boardcore::Canbus::CanData parseData(uint8_t sample)
    {
        Boardcore::Canbus::CanData tempData;
        tempData.length     = 1;
        tempData.payload[0] = sample;
        return tempData;
    }

private:
    miosix::FastMutex mutex;
    uint8_t percentage;
    bool updated;
};

}  // namespace common
