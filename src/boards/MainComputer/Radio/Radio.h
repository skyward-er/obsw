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

#pragma once

#include <mavlink_lib/pyxis/mavlink.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <radio/Xbee/Xbee.h>
#include <scheduler/TaskScheduler.h>

namespace Main
{

using MavDriver =
    Boardcore::MavlinkDriver<RADIO_PKT_LENGTH, RADIO_OUT_QUEUE_SIZE,
                             RADIO_MAV_MSG_LENGTH>;

class Radio
{
public:
    Boardcore::Xbee::Xbee* transceiver;

    Radio(Boardcore::TaskScheduler* scheduler);

    ~Radio();

    void handleMavlinkMessage(MavDriver* driver, const mavlink_message_t& msg);

    bool sendTelemetry(const uint8_t tmId);

private:
    void sendFlightTelemetry();

    void sendStatsTelemetry();
};

}  // namespace Main
