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

#include "Hub.h"

#include <Gs/Radio/Radio.h>
#include <Gs/Radio/RadioStatus.h>
#include <Gs/Ports/Serial.h>

using namespace Gs;
using namespace Boardcore;

void Hub::dispatchOutgoingMsg(const mavlink_message_t& msg) {
    RadioStatus *status = ModuleManager::getInstance().get<RadioStatus>();

    // TODO: Dispatch to correct radio using mavlink ids

    if (status->isMainRadioPresent())
    {
        RadioMain *radio = ModuleManager::getInstance().get<RadioMain>();
        radio->sendMsg(msg);
    }

    if (status->isPayloadRadioPresent())
    {
        RadioPayload *radio = ModuleManager::getInstance().get<RadioPayload>();
        radio->sendMsg(msg);
    }
}

void Hub::dispatchIncomingMsg(const mavlink_message_t& msg) {
    Serial* serial = ModuleManager::getInstance().get<Serial>();
    serial->sendMsg(msg);

    // TODO: Add UDP dispatch
}