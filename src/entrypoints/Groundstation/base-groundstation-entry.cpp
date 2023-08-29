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

#include <Groundstation/Base/Buses.h>
#include <Groundstation/Base/Hub.h>
#include <Groundstation/Base/Ports/Ethernet.h>
#include <Groundstation/Base/Radio/Radio.h>
#include <Groundstation/Base/Radio/RadioStatus.h>
#include <Groundstation/Common/Ports/Serial.h>
#include <miosix.h>

using namespace Groundstation;
using namespace Boardcore;
using namespace miosix;

void spinLoop()
{
    while (1)
    {
        Thread::sleep(1000);
    }
}

void errorLoop()
{
    while (1)
    {
        led1On();
        Thread::sleep(100);
        led1Off();
        Thread::sleep(100);
    }
}

int main()
{
    ledOff();

    Hub *hub                    = new Hub();
    Buses *buses                = new Buses();
    Serial *serial              = new Serial();
    Ethernet *ethernet          = new Ethernet();
    RadioMain *radio_main       = new RadioMain();
    RadioPayload *radio_payload = new RadioPayload();
    RadioStatus *radio_status   = new RadioStatus();

    ModuleManager &modules = ModuleManager::getInstance();

    bool ok = true;

    ok &= modules.insert<HubBase>(hub);
    ok &= modules.insert(buses);
    ok &= modules.insert(serial);
    ok &= modules.insert(ethernet);
    ok &= modules.insert(radio_main);
    ok &= modules.insert(radio_payload);
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

    ok &= ethernet->start();
    if (!ok)
    {
        printf("[error] Failed to start ethernet!\n");
    }

    ok &= radio_main->start();
    if (!ok)
    {
        printf("[error] Failed to start main radio!\n");
    }

    ok &= radio_payload->start();
    if (!ok)
    {
        printf("[error] Failed to start payload radio!\n");
    }

    ok &= radio_status->start();
    if (!ok)
    {
        printf("[error] Failed to start radio status!\n");
    }

    if (radio_status->isMainRadioPresent())
    {
        led2On();
    }

    if (radio_status->isPayloadRadioPresent())
    {
        led3On();
    }

    if (!ok)
    {
        errorLoop();
    }

    led1On();
    spinLoop();
    return 0;
}