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

#pragma once

#include <Groundstation/Common/HubBase.h>
#include <Groundstation/Common/Radio/RadioBase.h>
#include <Groundstation/Rovie/Buses.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <atomic>

namespace GroundstationRovie
{

using RadioMavDriver =
    Boardcore::MavlinkDriver<Boardcore::SX1278Lora::MTU,
                             Groundstation::MAV_OUT_QUEUE_SIZE,
                             MAVLINK_MAX_DIALECT_PAYLOAD_SIZE>;

class RadioRig
    : public Boardcore::InjectableWithDeps<Groundstation::HubBase, Buses>
{
public:
    RadioRig() {}

    [[nodiscard]] bool start();

private:
    /**
     * @brief Called internally when a message is received.
     */
    void handleMsg(const mavlink_message_t& msg);

    std::atomic<bool> started{false};

    // Objects are always destructed in reverse order, so keep them in this
    // order
    std::unique_ptr<Boardcore::SX1278Lora> sx1278;
    std::unique_ptr<RadioMavDriver> mavDriver;
};

}  // namespace GroundstationRovie