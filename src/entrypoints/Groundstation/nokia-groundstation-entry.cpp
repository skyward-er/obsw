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

#include <Groundstation/Common/Ports/Serial.h>
#include <Groundstation/Nokia/Buses.h>
#include <Groundstation/Nokia/Hub.h>
#include <Groundstation/Nokia/Radio/Radio.h>
#include <miosix.h>
#include <utils/DependencyManager/DependencyManager.h>

using namespace Groundstation;
using namespace GroundstationNokia;
using namespace Boardcore;
using namespace miosix;

void idleLoop()
{
    while (1)
    {
        Thread::wait();
    }
}

int main()
{
    ledOff();

    Hub *hub       = new Hub();
    Buses *buses   = new Buses();
    Radio *radio   = new Radio();
    Serial *serial = new Serial();

    DependencyManager manager;

    bool ok = true;

    ok &= manager.insert<HubBase>(hub);
    ok &= manager.insert(buses);
    ok &= manager.insert(serial);
    ok &= manager.insert(radio);

    // If insertion failed, stop right here
    if (!ok)
    {
        printf("[error] Failed to insert all modules!\n");
        idleLoop();
    }

    if (!manager.inject())
    {
        printf("[error] Failed to inject the dependencies!\n");
        idleLoop();
    }

    // Ok now start them

    ok &= serial->start();
    if (!ok)
    {
        printf("[error] Failed to start serial!\n");
    }

    ok &= radio->start();
    if (!ok)
    {
        printf("[error] Failed to start radio!\n");
    }

    if (ok)
    {
        printf("Init complete!\n");
    }

    idleLoop();
    return 0;
}