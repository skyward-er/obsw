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

using namespace Gs;
using namespace Boardcore;

void spinLoop() {
    while(1) {
        miosix::Thread::sleep(1000);
    }
}

int main()
{

    Buses *buses                = new Buses();
    Serial *serial              = new Serial();
    RadioMain *radio_main       = new RadioMain();
    // RadioPayload *radio_payload = new RadioPayload();
    RadioStatus *radio_status   = new RadioStatus();

    ModuleManager &modules = ModuleManager::getInstance();

    bool ok = true;

    ok &= modules.insert(buses);
    ok &= modules.insert(serial);
    ok &= modules.insert(radio_main);
    // ok &= modules.insert(radio_payload);
    ok &= modules.insert(radio_status);

    // If insertion failed, stop right here
    if(!ok) {
        spinLoop();
    }

    // Ok now start them

    ok &= serial->start();
    ok &= radio_main->start();
    // ok &= radio_payload->start();

    /*if(!ok) {
        printf("[GS] Init failed!\n");
    } else {
        printf("[GS] Init succesfull!\n");
        printf("[GS] radio main: %d\n", radio_status->isMainRadioPresent());
        printf("[GS] radio payload: %d\n",
    radio_status->isPayloadRadioPresent());
    }*/

    spinLoop();
    return 0;
}