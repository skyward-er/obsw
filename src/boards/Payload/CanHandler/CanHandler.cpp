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

#include "CanHandler.h"

#include <Payload/Actuators/Actuators.h>
#include <Payload/BoardScheduler.h>
#include <Payload/Configs/CanHandlerConfig.h>
#include <Payload/Sensors/Sensors.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>
#include <common/Events.h>
#include <events/EventBroker.h>

using namespace Boardcore;
using namespace Boardcore::Canbus;
using namespace Common;
using namespace Common::CanConfig;
namespace config = Payload::Config::CanHandler;

namespace Payload
{

using namespace std::chrono;

bool CanStatus::isMainConnected()
{
    return miosix::getTime() <=
           mainLastStatus + nanoseconds{config::Status::TIMEOUT}.count();
}

bool CanStatus::isRigConnected()
{
    return miosix::getTime() <=
           rigLastStatus + nanoseconds{config::Status::TIMEOUT}.count();
}

bool CanStatus::isMotorConnected()
{
    return miosix::getTime() <=
           motorLastStatus + nanoseconds{config::Status::TIMEOUT}.count();
}

bool CanHandler::start()
{
    auto& scheduler = getModule<BoardScheduler>()->canHandler();

    // Create CanbusDriver
    driver = std::make_unique<CanbusDriver>(CAN1, CanConfig::CONFIG,
                                            CanConfig::BIT_TIMING);

    // Create CanProtocol
    protocol = std::make_unique<CanProtocol>(
        driver.get(), [this](const auto& msg) { handleMessage(msg); },
        BoardScheduler::Priority::MEDIUM);

    // Add MAIN filter
    bool filterAdded =
        protocol->addFilter(static_cast<uint8_t>(Board::MAIN),
                            static_cast<uint8_t>(Board::BROADCAST));
    if (!filterAdded)
    {
        LOG_ERR(logger, "Failed to add MAIN filter");
        return false;
    }

    // Add RIG filter
    filterAdded = protocol->addFilter(static_cast<uint8_t>(Board::RIG),
                                      static_cast<uint8_t>(Board::BROADCAST));
    if (!filterAdded)
    {
        LOG_ERR(logger, "Failed to add RIG filter");
        return false;
    }

    // Add MOTOR filter
    filterAdded = protocol->addFilter(static_cast<uint8_t>(Board::MOTOR),
                                      static_cast<uint8_t>(Board::BROADCAST));
    if (!filterAdded)
    {
        LOG_ERR(logger, "Failed to add MOTOR filter");
        return false;
    }

    // Initialize CanbusDriver
    LOG_DEBUG(logger, "Initializing CanbusDriver, this may take a while...");
    driver->init();

    // Add the status task
    auto statusTask = scheduler.addTask(
        [this]
        {
            auto logStats = Logger::getInstance().getStats();
            auto state    = getModule<FlightModeManager>()->getStatus().state;
            auto status   = DeviceStatus{
                  .timestamp = TimestampTimer::getTimestamp(),
                  .logNumber = static_cast<int16_t>(logStats.logNumber),
                  .state     = static_cast<uint8_t>(state),
                  .armed     = state == FlightModeManagerState::ARMED,
                  .hil       = false,  // TODO: hil
                  .logGood   = logStats.lastWriteError == 0,
            };

            protocol->enqueueData(static_cast<uint8_t>(Priority::MEDIUM),
                                  static_cast<uint8_t>(PrimaryType::STATUS),
                                  static_cast<uint8_t>(Board::PAYLOAD),
                                  static_cast<uint8_t>(Board::BROADCAST), 0x00,
                                  status);
        },
        config::Status::PERIOD);

    if (statusTask == 0)
    {
        LOG_ERR(logger, "Failed to add periodic status task");
        return false;
    }

    // Add the pitot task
    auto pitotTask = scheduler.addTask(
        [this]()
        {
            protocol->enqueueData(static_cast<uint8_t>(Priority::MEDIUM),
                                  static_cast<uint8_t>(PrimaryType::SENSORS),
                                  static_cast<uint8_t>(Board::PAYLOAD),
                                  static_cast<uint8_t>(Board::BROADCAST),
                                  static_cast<uint8_t>(SensorId::PITOT),
                                  getModule<Sensors>()->getPitotLastSample());
        },
        config::Pitot::PERIOD);

    if (pitotTask == 0)
    {
        LOG_ERR(logger, "Failed to add periodic pitot task");
        return false;
    }

    bool protocolStarted = protocol->start();
    if (!protocolStarted)
    {
        LOG_ERR(logger, "Failed to start CanProtocol");
        return false;
    }

    started = true;
    return true;
}

bool CanHandler::isStarted() { return started; }

void CanHandler::sendEvent(EventId event)
{
    protocol->enqueueEvent(static_cast<uint8_t>(Priority::CRITICAL),
                           static_cast<uint8_t>(PrimaryType::EVENTS),
                           static_cast<uint8_t>(Board::PAYLOAD),
                           static_cast<uint8_t>(Board::BROADCAST),
                           static_cast<uint8_t>(event));
}

CanStatus CanHandler::getCanStatus()
{
    miosix::Lock<miosix::FastMutex> lock(statusMutex);
    return status;
}

void CanHandler::handleMessage(const CanMessage& msg)
{
    auto type = static_cast<PrimaryType>(msg.getPrimaryType());
    switch (type)
    {
        case PrimaryType::EVENTS:
        {
            handleEvent(msg);
            break;
        }

        case PrimaryType::STATUS:
        {
            handleStatus(msg);
            break;
        }

        default:
        {
            break;
        }
    }
}

void CanHandler::handleEvent(const Boardcore::Canbus::CanMessage& msg)
{
    auto canEvent = static_cast<EventId>(msg.getSecondaryType());
    auto event    = eventToEvent.find(canEvent);

    if (event == eventToEvent.end())
    {
        return;
    }

    EventBroker::getInstance().post(event->second, TOPIC_CAN);
}

void CanHandler::handleStatus(const CanMessage& msg)
{
    auto board        = static_cast<Board>(msg.getSource());
    auto deviceStatus = deviceStatusFromCanMessage(msg);

    miosix::Lock<miosix::FastMutex> lock(statusMutex);

    switch (board)
    {
        case Board::MAIN:
        {
            status.mainLastStatus = miosix::getTime();
            status.mainState      = deviceStatus.state;
            status.mainArmed      = deviceStatus.armed;
            break;
        }

        case Board::RIG:
        {
            status.rigLastStatus = miosix::getTime();
            status.rigState      = deviceStatus.state;
            status.rigArmed      = deviceStatus.armed;
            break;
        }

        case Board::MOTOR:
        {
            status.motorLastStatus = miosix::getTime();
            status.motorState      = deviceStatus.state;

            status.motorLogNumber = deviceStatus.logNumber;
            status.motorLogGood   = deviceStatus.logGood;
            status.motorHil       = deviceStatus.hil;
            break;
        }

        default:
        {
            break;
        }
    }
}

}  // namespace Payload
