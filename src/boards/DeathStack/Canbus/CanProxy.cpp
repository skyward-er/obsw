/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise De Faveri
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

#include <Common.h>
#include "CanProxy.h"

#include "CanInterfaces.h"
#include "DeathStack/events/Events.h"
#include "DeathStack/events/Topics.h"

#include <events/EventBroker.h>

namespace DeathStackBoard
{

using namespace std::placeholders;

/**
 * Canbus receiving function.
 * @param message    the received message to be handled
 * @param proxy      the object that has the reference to the bus
 */
static void canRcv(const CanMsg& message, const CanStatus& status) 
{
    TRACE("[CAN] Received message with id %lu\n", message.StdId);

    /* Create event */
    CanbusEvent ev;
    ev.sig = EV_NEW_CAN_MSG;
    ev.canTopic = message.StdId;
    ev.len = message.DLC;
    memcpy(ev.payload, message.Data, CAN_MAX_PAYLOAD);

    /* Log stats */
    LoggerService::getInstance()->log(status);

    /* Post event */
    sEventBroker->post(ev, TOPIC_CAN);
}

/**
 * Initialise CAN1 on PA11, PA12, set filters and set receiver function.
 */
CanProxy::CanProxy(CanManager* c)
{
    // Init structure (pins)
    canbus_init_t st = {
        CAN1, miosix::Mode::ALTERNATE, 9, {CAN1_RX0_IRQn, CAN1_RX1_IRQn}};

    // Bus
    c->addBus<GPIOA_BASE, 11, 12>(st, &canRcv);
    bus = c->getBus(0);

    // Filters
    c->addHWFilter(CanInterfaces::CAN_TOPIC_IGNITION, 0);
    c->addHWFilter(CanInterfaces::CAN_TOPIC_NOSECONE, 0);

    TRACE("[CAN] Initialised CAN1 on PA11-12 \n");
}

/**
 * Canbus receiving function.
 */
bool CanProxy::send(uint16_t id, const uint8_t* message, uint8_t len)
{
    // Send
    bool ok = bus->send(id, message, len);

    // Log stats
    CanStatus status = bus->getStatus();
    LoggerService::getInstance()->log(status);

    return ok;
}

} /* namespace DeathStackBoard */
