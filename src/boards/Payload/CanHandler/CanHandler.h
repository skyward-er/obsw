/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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
#include <common/Mavlink.h>
#include <drivers/canbus/CanProtocol/CanProtocol.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Payload
{
class BoardScheduler;
class Sensors;
class FlightModeManager;
class Actuators;

struct CanStatus
{
    int64_t mainLastStatus  = 0;  ///< Timestamp of last main status message
    int64_t rigLastStatus   = 0;  ///< Timestamp of last rig status message
    int64_t motorLastStatus = 0;  ///< Timestamp of last motor status message

    uint8_t mainState  = 0;
    uint8_t rigState   = 0;
    uint8_t motorState = 0;

    bool mainArmed = false;
    bool rigArmed  = false;

    int16_t motorLogNumber = 0;
    bool motorLogGood      = true;
    bool motorHil          = false;

    bool isMainConnected();
    bool isRigConnected();
    bool isMotorConnected();
};

class CanHandler
    : public Boardcore::InjectableWithDeps<BoardScheduler, Sensors,
                                           FlightModeManager, Actuators>
{
public:
    [[nodiscard]] bool start();

    bool isStarted();

    void sendEvent(Common::CanConfig::EventId event);

    /**
     * @brief Send a servo open command over CAN.
     * @param servo Servo to open
     * @param openingTime Time to open the servo [ms]
     */
    void sendServoOpenCommand(ServosList servo, uint32_t openingTime);

    CanStatus getCanStatus();

private:
    void handleMessage(const Boardcore::Canbus::CanMessage &msg);
    void handleEvent(const Boardcore::Canbus::CanMessage &msg);
    void handleStatus(const Boardcore::Canbus::CanMessage &msg);

    std::unique_ptr<Boardcore::Canbus::CanbusDriver> driver;
    std::unique_ptr<Boardcore::Canbus::CanProtocol> protocol;

    CanStatus status;
    miosix::FastMutex statusMutex;

    std::atomic<bool> started{false};

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("CanHandler");
};

}  // namespace Payload
