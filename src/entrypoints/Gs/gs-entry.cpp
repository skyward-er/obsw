/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <Gs/Buses.h>
#include <Gs/Ports/Serial.h>
#include <Gs/Radio/Radio.h>
#include <Gs/Radio/RadioStatus.h>
#include <miosix.h>

using namespace Gs;
using namespace Boardcore;

// FIXME: Temporary hack waiting for IRQ fix.
// #define DISABLE_MAIN_RADIO
#define DISABLE_PAYLOAD_RADIO

void spinLoop()
{
    while (1)
    {
        miosix::Thread::sleep(1000);
    }
}

void errorLoop()
{
    while (1)
    {
        miosix::led1On();
        miosix::Thread::sleep(100);
        miosix::led1Off();
        miosix::Thread::sleep(100);
    }
}

int main()
{
    miosix::ledOff();

    Buses *buses   = new Buses();
    Serial *serial = new Serial();
#ifndef DISABLE_MAIN_RADIO
    RadioMain *radio_main = new RadioMain();
#endif
#ifndef DISABLE_PAYLOAD_RADIO
    RadioPayload *radio_payload = new RadioPayload();
#endif
    RadioStatus *radio_status = new RadioStatus();

    ModuleManager &modules = ModuleManager::getInstance();

    bool ok = true;

    ok &= modules.insert(buses);
    ok &= modules.insert(serial);
#ifndef DISABLE_MAIN_RADIO
    ok &= modules.insert(radio_main);
#endif
#ifndef DISABLE_PAYLOAD_RADIO
    ok &= modules.insert(radio_payload);
#endif
    ok &= modules.insert(radio_status);

    // If insertion failed, stop right here
    if (!ok)
    {
        printf("[error] Failed to insert all modules!\n");
        errorLoop();
    }

    // Ok now start them

    ok &= serial->start();
    if (!ok)
    {
        printf("[error] Failed to start serial!\n");
    }

#ifndef DISABLE_MAIN_RADIO
    ok &= radio_main->start();
    if (!ok)
    {
        printf("[error] Failed to start main radio!\n");
    }
#endif

#ifndef DISABLE_PAYLOAD_RADIO
    ok &= radio_payload->start();
    if (!ok)
    {
        printf("[error] Failed to start payload radio!\n");
    }
#endif

    ok &= radio_status->start();
    if (!ok)
    {
        printf("[error] Failed to start radio status!\n");
    }

    if (radio_status->isMainRadioPresent())
    {
        miosix::led2On();
    }

    if (radio_status->isPayloadRadioPresent())
    {
        miosix::led3On();
    }

    if (!ok)
    {
        errorLoop();
    }

    miosix::led1On();
    spinLoop();
    return 0;
}