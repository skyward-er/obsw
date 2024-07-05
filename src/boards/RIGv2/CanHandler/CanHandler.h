/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

#include <RIGv2/BoardScheduler.h>
#include <RIGv2/Sensors/Sensors.h>
#include <common/CanConfig.h>
#include <common/Mavlink.h>
#include <drivers/canbus/CanProtocol/CanProtocol.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace RIGv2
{

class Actuators;
class GroundModeManager;

class CanHandler
    : public Boardcore::InjectableWithDeps<BoardScheduler, GroundModeManager,
                                           Actuators, Sensors>
{
public:
    struct CanStatus
    {
        int mainCounter    = 0;
        int payloadCounter = 0;
        int motorCounter   = 0;

        bool mainArmed    = false;
        bool payloadArmed = false;

        bool isMainConnected() { return mainCounter == 0; }

        bool isPayloadConnected() { return payloadCounter == 0; }

        bool isMotorConnected() { return motorCounter == 0; }

        bool isMainArmed() { return mainArmed; }

        bool isPayloadArmed() { return payloadArmed; }
    };

    CanHandler();

    [[nodiscard]] bool start();

    bool isStarted();

    void sendEvent(Common::CanConfig::EventId event);

    void sendServoOpenCommand(ServosList servo, float maxAperture,
                              uint32_t openingTime);
    void sendServoCloseCommand(ServosList servo);

    CanStatus getCanStatus();

private:
    void handleMessage(const Boardcore::Canbus::CanMessage &msg);
    void handleEvent(const Boardcore::Canbus::CanMessage &msg);
    void handleSensor(const Boardcore::Canbus::CanMessage &msg);
    void handleActuator(const Boardcore::Canbus::CanMessage &msg);
    void handleStatus(const Boardcore::Canbus::CanMessage &msg);

    void periodicMessage();

    Boardcore::Canbus::CanbusDriver driver;
    Boardcore::Canbus::CanProtocol protocol;

    miosix::FastMutex statusMutex;
    CanStatus status;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("canhandler");
};

}  // namespace RIGv2