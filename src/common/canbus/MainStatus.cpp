/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Riccardo Sironi
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

#include "MainStatus.h"

#include <common/CanConfig.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>

using namespace Boardcore;

namespace Common
{
void MainStatus::handleCanMessage(const Canbus::CanMessage& msg)
{
    auto type = static_cast<CanConfig::PrimaryType>(msg.getPrimaryType());

    miosix::Lock<miosix::FastMutex> lock(mutex);

    switch (type)
    {
        case CanConfig::PrimaryType::SENSORS:
        {
            handleSensors(msg);
            break;
        }

        case CanConfig::PrimaryType::STATUS:
        {
            lastStatus  = Clock::now();
            data.device = deviceStatusFromCanMessage(msg);
            break;
        }

        case CanConfig::PrimaryType::ACTUATORS:
        {
            handleActuators(msg);
            break;
        }

        default:
            break;
    }
}

void MainStatus::handleSensors(const Canbus::CanMessage& msg)
{
    auto sensor = static_cast<CanConfig::AlgoId>(msg.getSecondaryType());

    switch (sensor)
    {
        case CanConfig::AlgoId::NAS_VERTICAL_SPEED:
        {
            auto algoData       = algoDataFromCanMessage(msg);
            data.nasVd          = algoData.value;
            data.nasTimestamp   = algoData.timestamp;
            data.nasRxTimestamp = TimestampTimer::getTimestamp();
            sdLogger.log(algoData);
            break;
        }

        case CanConfig::AlgoId::NAS_ALT_MSL:
        {
            auto algoData       = algoDataFromCanMessage(msg);
            data.nasAltitudeMsl = algoData.value;
            data.nasTimestamp   = algoData.timestamp;
            data.nasRxTimestamp = TimestampTimer::getTimestamp();
            sdLogger.log(algoData);
            break;
        }

        case CanConfig::AlgoId::NAS_STATE:
        {
            auto algoData = algoDataFromCanMessage(msg);
            data.nasState = nasControllerStateFromCanValue(
                static_cast<uint8_t>(algoData.value));
            data.nasStateTimestamp = algoData.timestamp;
            sdLogger.log(algoData);
            break;
        }

        case CanConfig::AlgoId::MEA_SHADOW_MODE_DELAY:
        {
            auto algoMillisData = algoMillisDataFromCanMessage(msg);
            data.shadowModeDelayMs =
                std::chrono::milliseconds{algoMillisData.value};
            data.shadowModeDelayTimestamp   = algoMillisData.timestamp;
            data.shadowModeDelayRxTimestamp = TimestampTimer::getTimestamp();
            sdLogger.log(algoMillisData);
            break;
        }

        default:
            break;
    }
}

void MainStatus::handleActuators(const Canbus::CanMessage& msg) { (void)msg; }

}  // namespace Common
