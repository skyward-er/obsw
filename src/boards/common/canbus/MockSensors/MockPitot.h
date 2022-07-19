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
#include <kernel/sync.h>
#include <sensors/SensorData.h>

namespace common
{

class MockPitot
{
public:
    MockPitot() {}

    // TODO: update to use pressure data
    Boardcore::PressureData getData()
    {
        miosix::Lock<miosix::FastMutex> l(mutex);
        updated = false;
        return data;
    }

    bool isUpdated()
    {
        miosix::Lock<miosix::FastMutex> l(mutex);
        return updated;
    }

    bool waitTillUpdated()
    {
        miosix::Lock<miosix::FastMutex> l(mutex);
        conVar.wait(l);
        return updated;
    }
    void setData(Boardcore::Canbus::CanData packet)
    {
        // Pressure and timestamp coded into an u_int64

        miosix::Lock<miosix::FastMutex> l(mutex);
        uint32_t temp = packet.payload[0] >> 32;
        memcpy(&data.pressure, &temp, sizeof(data.pressure));
        data.pressureTimestamp = (packet.payload[0] & 0xfffffff) << 2;
        updated                = true;
        conVar.broadcast();
    }

    Boardcore::Canbus::CanData parseData(Boardcore::PressureData sample)
    {
        // Pressure and timestamp coded into an u_int64

        Boardcore::Canbus::CanData tempData;
        tempData.length = 1;
        uint64_t temp;
        memcpy(&temp, &(sample.pressure), sizeof(sample.pressure));
        tempData.payload[0] = (temp << 32) | (sample.pressureTimestamp >> 2);
        return tempData;
    }

private:
    miosix::FastMutex mutex;
    miosix::ConditionVariable conVar;
    Boardcore::PressureData data;
    bool updated = false;
};

}  // namespace common
