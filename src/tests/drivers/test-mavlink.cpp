/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron, Nuno Barcellos
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

#include <boards/DeathStack/configs/TMTCConfig.h>

#include <Common.h>
#include <drivers/Xbee/Xbee.h>
#include <drivers/mavlink/multi/MavManager.h>
#include "DeathStack/TMTCManager/XbeeInterrupt.h"

#include "DeathStack/events/Events.h"
#include "events/FSM.h"

using namespace miosix;
using namespace DeathStackBoard;

Xbee_t* device;

int main()
{
    enableXbeeInterrupt();
    busSPI2::init();

    device = new Xbee_t();
    device->start();

    while(1)
    {
        TRACE("[TmtcTest] Sending ping\n");

        // Create a Mavlink message
        mavlink_message_t pingMsg;
        mavlink_msg_ping_tc_pack(1, 1, &pingMsg, miosix::getTick());

        uint8_t buff[100];
        int msgLen = mavlink_msg_to_send_buffer(buff, &pingMsg);

        for(int i = 0; i < msgLen; i++)
        {
            printf("Sending 0x%2x\n", buff[i]);
        }
        
        //uint8_t buff[5] = {'A', 'B', 'C', 'D', 'E'};

        // Send the message
        device->send(buff, msgLen);

        // ledOn();
        // miosix::delayMs(200);
        // ledOff();

        miosix::Thread::sleep(1000);
    }

    return 0;
}