/* Copyright (c) 2024 Skyward Experimental Rocketry
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

#include <Motor/BoardScheduler.h>
#include <Motor/Sensors/Sensors.h>
#include <common/CanConfig.h>
#include <drivers/canbus/CanProtocol/CanProtocol.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <atomic>

namespace Motor
{

class Actuators;

enum class InitStatus : uint8_t
{
    UNKNOWN  = 0,
    INIT_ERR = 1,
    INIT_OK  = 2,
};

class CanHandler
    : public Boardcore::InjectableWithDeps<BoardScheduler, Sensors, Actuators>
{
public:
    CanHandler();

    [[nodiscard]] bool start();

    void setInitStatus(InitStatus status);

private:
    void handleMessage(const Boardcore::Canbus::CanMessage &msg);
    void handleEvent(const Boardcore::Canbus::CanMessage &msg);
    void handleCommand(const Boardcore::Canbus::CanMessage &msg);

    Boardcore::Logger &sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("canhandler");

    std::atomic<InitStatus> initStatus{InitStatus::UNKNOWN};

    Boardcore::Canbus::CanbusDriver driver;
    Boardcore::Canbus::CanProtocol protocol;
};

}  // namespace Motor
