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

// Ignore warnings as these are auto-generated headers made by a third party
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include <mavlink_lib/pyxis/mavlink.h>
#pragma GCC diagnostic pop

#include <Main/Configs/RadioConfigs.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <radio/SerialTransceiver/SerialTransceiver.h>
#include <scheduler/TaskScheduler.h>

namespace Main
{

using MavDriver = Boardcore::MavlinkDriver<RadioConfigs::RADIO_PKT_LENGTH,
                                           RadioConfigs::RADIO_OUT_QUEUE_SIZE,
                                           RadioConfigs::RADIO_MAV_MSG_LENGTH>;

class Radio
{
public:
    explicit Radio(Boardcore::TaskScheduler* scheduler);

    /**
     * @brief Called by the MavlinkDriver when a message is received.
     *
     * @param driver Mavlink driver who calls the function.
     * @param msg Parsed message.
     */
    void handleMavlinkMessage(MavDriver* driver, const mavlink_message_t& msg);

    /**
     * @brief Used to send the specified telemetry message.
     *
     * @param tmId Identifier for the message to send.
     * @return boolean that indicates the operation's result.
     */
    bool sendTelemetry(const uint8_t tmId);

private:
    Boardcore::Transceiver* transceiver;
    MavDriver* mavDriver;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("radio");
};

}  // namespace Main
