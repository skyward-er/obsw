/* Copyright (c) 2022 Skyward Experimental Rocketry
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
#include <common/canbus/CanHandler.h>
#include <common/canbus/SensorMocap/AereoBrakes.h>
#include <common/canbus/SensorMocap/PitotMocap.h>

#include "drivers/canbus/BusLoadEstimation.h"
#include "drivers/canbus/Canbus.h"
#include "utils/collections/CircularBuffer.h"

constexpr uint32_t BAUD_RATE = 500 * 1000;
constexpr float SAMPLE_POINT = 87.5f / 100.0f;

using namespace Boardcore;
using namespace Boardcore::Canbus;
using namespace miosix;
using namespace common;

#ifdef _ARCH_CORTEXM3_STM32
using CanRX = Gpio<GPIOA_BASE, 11>;
using CanTX = Gpio<GPIOA_BASE, 12>;
#else
using CanRX = Gpio<GPIOA_BASE, 11>;
using CanTX = Gpio<GPIOA_BASE, 12>;
#endif

int main()
{

    {
        miosix::FastInterruptDisableLock dLock;

#ifdef _ARCH_CORTEXM3_STM32
        CanRX::mode(Mode::ALTERNATE);
        CanTX::mode(Mode::ALTERNATE);
#else
        CanRX::mode(Mode::ALTERNATE);
        CanTX::mode(Mode::ALTERNATE);

        CanRX::alternateFunction(9);
        CanTX::alternateFunction(9);
#endif
    }

    CanbusDriver::CanbusConfig cfg{};
    CanbusDriver::AutoBitTiming bt;
    bt.baudRate    = BAUD_RATE;
    bt.samplePoint = SAMPLE_POINT;

    CanbusDriver* c = new CanbusDriver(CAN1, cfg, bt);
    PitotMocap* p   = new PitotMocap();
    AereoBrakes* a  = new AereoBrakes();
    CanHandler handler(c, Boards::Main, p, a);

    // Allow every message
    Mask32FilterBank f2(0, 0, 0, 0, 0, 0, 0);

    c->addFilter(f2);
    c->init();

    handler.start();
    // send event, data and command
    const int slp = 500;
    CanData toSend;
    toSend.canId      = 0x01;
    toSend.len        = 3;
    toSend.payload[0] = 1;
    toSend.payload[1] = 2;
    toSend.payload[2] = 3;
    for (;;)
    {

        Thread::sleep(slp);
    }
}
