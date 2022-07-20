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

#include <common/canbus/CanConfig.h>
#include <drivers/canbus/CanProtocol.h>
#include <kernel/sync.h>

namespace common
{

class MockSensor
{
public:
    virtual void put(Boardcore::Canbus::CanData packet) = 0;
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

    SensorID getID() { return id; }

protected:
    MockSensor(SensorID i) : id(i){};
    miosix::FastMutex mutex;
    miosix::ConditionVariable conVar;
    bool updated = false;
    SensorID id;
};
}  // namespace common
