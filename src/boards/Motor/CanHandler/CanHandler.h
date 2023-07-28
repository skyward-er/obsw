/* Copyright (c) 2023 Skyward Experimental Rocketry
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
#include <scheduler/TaskScheduler.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Motor
{

class CanHandler : public Boardcore::Module
{

public:
    explicit CanHandler(Boardcore::TaskScheduler *sched);

    /**
     * @brief Adds the periodic task to the scheduler and starts the protocol
     * threads
     */
    bool start();

    /**
     * @brief Returns true if the protocol threads are started and the scheduler
     * is running
     */
    bool isStarted();

    /**
     * @brief Sends a CAN event on the bus
     */
    void sendEvent(Common::CanConfig::EventId event);

    /**
     * @brief Set the initialization flag to true
     */
    void setInitStatus(bool initResult);

private:
    /**
     * @brief Handles a generic CAN message and dispatch the message to the
     * correct handler
     */
    void handleCanMessage(const Boardcore::Canbus::CanMessage &msg);

    // CAN message handlers
    void handleCanEvent(const Boardcore::Canbus::CanMessage &msg);
    void handleCanCommand(const Boardcore::Canbus::CanMessage &msg);

    // Init status
    std::atomic<bool> initStatus{false};

    // CAN interfaces
    Boardcore::Canbus::CanbusDriver *driver;
    Boardcore::Canbus::CanProtocol *protocol;

    Boardcore::TaskScheduler *scheduler;
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("canhandler");
};

}  // namespace Motor
