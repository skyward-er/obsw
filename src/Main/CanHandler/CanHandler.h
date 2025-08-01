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

#include <Main/BoardScheduler.h>
#include <Main/PersistentVars/PersistentVars.h>
#include <Main/Sensors/Sensors.h>
#include <common/CanConfig.h>
#include <common/MavlinkOrion.h>
#include <common/canbus/MotorStatus.h>
#include <drivers/canbus/CanProtocol/CanProtocol.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <chrono>

namespace Main
{

class FlightModeManager;
class Actuators;

class CanHandler
    : public Boardcore::InjectableWithDeps<BoardScheduler, Actuators, Sensors,
                                           FlightModeManager,
                                           Common::MotorStatus>
{
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

public:
    struct CanStatus
    {
        TimePoint rigLastStatus     = {};
        TimePoint payloadLastStatus = {};

        uint8_t rigState     = 0;
        uint8_t payloadState = 0;

        bool rigArmed     = false;
        bool payloadArmed = false;

        bool isRigConnected()
        {
            return Clock::now() <=
                   rigLastStatus + Common::CanConfig::STATUS_TIMEOUT;
        }
        bool isPayloadConnected()
        {
            return Clock::now() <=
                   payloadLastStatus + Common::CanConfig::STATUS_TIMEOUT;
        }

        uint8_t getRigState() { return rigState; }
        uint8_t getPayloadState() { return payloadState; }

        bool isRigArmed() { return rigArmed; }
        bool isPayloadArmed() { return payloadArmed; }
    };

    CanHandler();

    [[nodiscard]] bool start();

    bool isStarted();

    void sendEvent(Common::CanConfig::EventId event);

    void sendServoOpenCommand(ServosList servo, uint32_t openingTime);
    void sendServoCloseCommand(ServosList servo);

    CanStatus getCanStatus();

private:
    void handleMessage(const Boardcore::Canbus::CanMessage& msg);
    void handleEvent(const Boardcore::Canbus::CanMessage& msg);
    void handleSensor(const Boardcore::Canbus::CanMessage& msg);
    void handleActuator(const Boardcore::Canbus::CanMessage& msg);
    void handleStatus(const Boardcore::Canbus::CanMessage& msg);

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("canhandler");
    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();

    std::atomic<bool> started{false};

    Boardcore::Canbus::CanbusDriver driver;
    Boardcore::Canbus::CanProtocol protocol;

    miosix::FastMutex statusMutex;
    CanStatus status;
};

}  // namespace Main
