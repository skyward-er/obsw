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
#include <sensors/SensorData.h>

namespace common
{

class PitotMocap
{
public:
    PitotMocap() {}

    Boardcore::PressureData GetData()  // todo update to use pressure data
    {
        m.lock();
        updated = false;
        m.unlock();
        return data;
    }

    bool Updated() { return updated; }

    void SetData(Boardcore::Canbus::CanData packet)
    {
        // the pitot packet is a the first 4 byte pressure and the last 4
        // timestamp
        m.lock();
        data.pressure          = packet.payload[0] >> 32;
        data.pressureTimestamp = packet.payload[0] & 0xffffffff;

        updated = true;
        m.unlock();
    }

    Boardcore::Canbus::CanData ParseData(
        Boardcore::PressureData
            sample)  // code pressure and timestamp into an u_int64
    {
        Boardcore::Canbus::CanData tempData;
        tempData.len = 1;
        uint32_t temp;
        memcpy(&temp, &(sample.pressure), sizeof(temp));
        tempData.payload[0] = temp << 32 & sample.pressureTimestamp >> 2;
        return tempData;
    }

private:
    bool updated = false;
    Boardcore::PressureData data;
    miosix::FastMutex m;
};
}  // namespace common
