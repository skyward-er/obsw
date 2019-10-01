/* Copyright (c) 2018-2019 Skyward Experimental Rocketry
 * Authors: Alvise de' Faveri Tron
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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISIN\G FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <boards/DeathStack/FlightModeManager/FlightModeManager.h>
#include <skyward-boardcore/src/shared/utils/EventSniffer.h>

#include <boards/DeathStack/events/Events.h>
#include <boards/DeathStack/events/Topics.h>

#include <inttypes.h>

using namespace miosix;
using namespace DeathStackBoard;

void printEvent(uint8_t event, uint8_t topic)
{
    TRACE("%s on %s\n", getEventString(event).c_str(), getTopicString(topic).c_str());
}
int main()
{
    FlightModeManager* fmm = new FlightModeManager();
    fmm->start();
    sEventBroker->start();

    EventSniffer* sniffer = new EventSniffer(*sEventBroker, TOPIC_LIST, printEvent);
    UNUSED(sniffer); // The sniffer's handler will be called by evBroker

    printf("\nOk\n");

    Event ev;
    int topic;

    while(true)
    {
        printf("Choose an event\n");
        printf("ID:\n");
        scanf("%hu", &(ev.sig));
        printf("TOPIC:\n");
        scanf("%d", &topic);

        sEventBroker->post(ev, topic);
    }
    
    return 0;
}