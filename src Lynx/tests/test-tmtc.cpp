/* Copyright (c) 2018-2019 Skyward Experimental Rocketry
 * Author: Alvise de'Faveri Tron
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

#include <TMTCManager/TMTCManager.h>
#include <events/Events.h>
#include <events/Topics.h>
#include <interfaces-impl/hwmapping.h>
#include <skyward-boardcore/src/shared/utils/EventSniffer.h>

using namespace miosix;
using namespace DeathStackBoard;

void onEventReceived(uint8_t event, uint8_t topic)
{
    TRACE("%s on %s\n", getEventString(event).c_str(),
          getTopicString(topic).c_str());
}

int main()
{
    busSPI2::init();

    TMTCController* tmtc = new TMTCController();
    tmtc->start();
    sEventBroker.start();

    Thread::sleep(1000);

    printf("\nOk, press open to post liftoff...\n");
    while (inputs::btn_open::value())
        ;

    Thread::sleep(100);

    sEventBroker.post({EV_LIFTOFF}, TOPIC_FLIGHT_EVENTS);

    printf("Press close to reboot...\n");
    while (inputs::btn_close::value())
        ;

    miosix::reboot();
}
