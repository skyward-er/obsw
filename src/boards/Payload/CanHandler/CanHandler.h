/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Federico Mandelli, Alberto Nidasio
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

#include <common/CanConfig.h>
#include <drivers/canbus/CanProtocol/CanProtocol.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Payload
{
class BoardScheduler;
class Sensors;
class FlightModeManager;

/**
 * @class CanHandler
 *
 * @brief Payload implementation of CanProtocol with methods for ease of use
 *
 */
class CanHandler : public Boardcore::InjectableWithDeps<BoardScheduler, Sensors,
                                                        FlightModeManager>
{
public:
    /**
     * @brief Creates the protocol and the driver and adds the filters.
     *
     * @warning With Payload motherboard CanDriver initialization could be
     * blocking
     */
    CanHandler();

    /**
     * @brief Starts the protocol and adds the periodic messages to the task
     * scheduler.
     *
     * @return true if the protocol started correctly and the tasks were
     * successfully inserted
     */
    bool start();

    /**
     * @return true if the protocol is started
     */
    bool isStarted();

    /**
     * @brief Compile the the ID of the message and send the event trough
     * CanProtocol
     */
    void sendEvent(Common::CanConfig::EventId event);

private:
    /**
     * @brief Function called by CanProtocol when a new message is received
     */
    void handleCanMessage(const Boardcore::Canbus::CanMessage& msg);

    /**
     * @brief Converts the received CanEvent in Event and post it on TOPIC_CAN
     */
    void handleCanEvent(const Boardcore::Canbus::CanMessage& msg);

    Boardcore::Canbus::CanbusDriver* driver;
    Boardcore::Canbus::CanProtocol* protocol;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("canhandler");
};

}  // namespace Payload
