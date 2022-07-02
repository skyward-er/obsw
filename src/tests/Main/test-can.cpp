/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <drivers/canbus/Canbus.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;
using namespace Boardcore::Canbus;

CanbusDriver::AutoBitTiming bt{
    500000,         // Set baud rate in bits per second
    87.5f / 100.0f  // Set sample point (in percentage of the bit length)
};

int main()
{
    CanbusDriver can(CAN1, {}, bt);
    can.init();

    while (true)
    {
        CanPacket p;
        p.id      = 12345;  // Message identifier
        p.ext     = true;   // Extended identifier message
        p.length  = 1;      // Length of the payload
        p.data[0] = 42;

        can.send(p);
        printf("Sent message\n");

        // // Check if we have received a packet
        // if (!can.getRXBuffer().isEmpty())
        // {
        //     // Remove it from the buffer for further processing
        //     Canbus::CanRXPacket prx = can.getRXBuffer().pop();

        //     printf("Received new packet! Payload: %d\n", prx.packet.data[0]);
        // }

        delayMs(1000);
    }
}