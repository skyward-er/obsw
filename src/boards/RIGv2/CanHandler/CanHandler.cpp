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

#include "CanHandler.h"

#include <RIGv2/Actuators/Actuators.h>
#include <RIGv2/Configs/CanHandlerConfig.h>
#include <RIGv2/Configs/SchedulerConfig.h>
#include <RIGv2/StateMachines/GroundModeManager/GroundModeManager.h>

using namespace miosix;
using namespace RIGv2;
using namespace Boardcore;
using namespace Boardcore::Canbus;
using namespace Common;

static constexpr CanbusDriver::CanbusConfig CONFIG;
static constexpr CanbusDriver::AutoBitTiming BIT_TIMING = {
    .baudRate = CanConfig::BAUD_RATE, .samplePoint = CanConfig::SAMPLE_POINT};

CanHandler::CanHandler()
    : driver(CAN1, CONFIG, BIT_TIMING),
      protocol(
          &driver, [this](const CanMessage &msg) { handleMessage(msg); },
          Config::Scheduler::CAN_PRIORITY)
{
    protocol.addFilter(static_cast<uint8_t>(CanConfig::Board::MAIN),
                       static_cast<uint8_t>(CanConfig::Board::BROADCAST));
    protocol.addFilter(static_cast<uint8_t>(CanConfig::Board::MOTOR),
                       static_cast<uint8_t>(CanConfig::Board::BROADCAST));
    protocol.addFilter(static_cast<uint8_t>(CanConfig::Board::PAYLOAD),
                       static_cast<uint8_t>(CanConfig::Board::BROADCAST));
}

bool CanHandler::start()
{
    driver.init();

    TaskScheduler &scheduler =
        getModule<BoardScheduler>()->getCanBusScheduler();

    uint8_t result = scheduler.addTask([this]() { periodicMessage(); },
                                       Config::CanHandler::STATUS_PERIOD);

    return result != 0 && protocol.start();
}

void CanHandler::sendEvent(CanConfig::EventId event)
{
    protocol.enqueueEvent(static_cast<uint8_t>(CanConfig::Priority::CRITICAL),
                          static_cast<uint8_t>(CanConfig::PrimaryType::EVENTS),
                          static_cast<uint8_t>(CanConfig::Board::RIG),
                          static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                          static_cast<uint8_t>(event));
}

void CanHandler::sendServoOpenCommand(ServosList servo, float maxAperture,
                                      uint32_t openingTime)
{
    uint32_t maxApertureBits = 0;
    std::memcpy(&maxApertureBits, &maxAperture, sizeof(maxAperture));

    uint64_t payload = (static_cast<uint64_t>(openingTime) << 32) |
                       static_cast<uint64_t>(maxApertureBits);

    protocol.enqueueSimplePacket(
        static_cast<uint8_t>(CanConfig::Priority::CRITICAL),
        static_cast<uint8_t>(CanConfig::PrimaryType::COMMAND),
        static_cast<uint8_t>(CanConfig::Board::RIG),
        static_cast<uint8_t>(CanConfig::Board::BROADCAST),
        static_cast<uint8_t>(servo), payload);
}

CanHandler::CanStatus CanHandler::getCanStatus()
{
    Lock<FastMutex> lock{statusMutex};
    return status;
}

void CanHandler::sendServoCloseCommand(ServosList servo)
{
    // Closing a servo means opening it for 0s
    sendServoOpenCommand(servo, 0.0f, 0);
}

void CanHandler::handleMessage(const Boardcore::Canbus::CanMessage &msg)
{
    CanConfig::PrimaryType type =
        static_cast<CanConfig::PrimaryType>(msg.getPrimaryType());

    switch (type)
    {
        case CanConfig::PrimaryType::EVENTS:
        {
            handleEvent(msg);
            break;
        }

        case CanConfig::PrimaryType::SENSORS:
        {
            handleSensor(msg);
            break;
        }

        case CanConfig::PrimaryType::STATUS:
        {
            handleStatus(msg);
            break;
        }

        case CanConfig::PrimaryType::ACTUATORS:
        {
            handleActuator(msg);
            break;
        }

        default:
        {
            LOG_WARN(logger, "Received unsupported message type: {}", type);
            break;
        }
    }
}

void CanHandler::handleEvent(const Boardcore::Canbus::CanMessage &msg)
{
    CanConfig::EventId event =
        static_cast<CanConfig::EventId>(msg.getSecondaryType());
    LOG_WARN(logger, "Received unrecognized event: {}", event);
}

void CanHandler::handleSensor(const Boardcore::Canbus::CanMessage &msg)
{
    CanConfig::SensorId sensor =
        static_cast<CanConfig::SensorId>(msg.getSecondaryType());

    Sensors *sensors = getModule<Sensors>();
    switch (sensor)
    {
        case CanConfig::SensorId::CC_PRESSURE:
        {
            PressureData data = pressureDataFromCanMessage(msg);
            sensors->setCanCCPress(data);
            break;
        }

        case CanConfig::SensorId::BOTTOM_TANK_PRESSURE:
        {
            PressureData data = pressureDataFromCanMessage(msg);
            sensors->setCanBottomTankPress(data);
            break;
        }

        case CanConfig::SensorId::TOP_TANK_PRESSURE:
        {
            PressureData data = pressureDataFromCanMessage(msg);
            sensors->setCanTopTankPress(data);
            break;
        }

        case CanConfig::SensorId::TANK_TEMPERATURE:
        {
            TemperatureData data = temperatureDataFromCanMessage(msg);
            sensors->setCanTankTemp(data);
            break;
        }

        default:
        {
            LOG_WARN(logger, "Received unsupported sensor data: {}", sensor);
        }
    }
}

void CanHandler::handleActuator(const Boardcore::Canbus::CanMessage &msg)
{
    ServosList servo = static_cast<ServosList>(msg.getSecondaryType());
    ServoData data   = servoDataFromCanMessage(msg);

    getModule<Actuators>()->setCanServoAperture(servo, data.position);
}

void CanHandler::handleStatus(const Boardcore::Canbus::CanMessage &msg)
{
    CanConfig::Board source = static_cast<CanConfig::Board>(msg.getSource());

    bool armed = msg.payload[0] != 0;

    Lock<FastMutex> lock{statusMutex};

    switch (source)
    {
        case CanConfig::Board::MAIN:
        {
            status.mainCounter = 4;
            status.mainArmed   = armed;
            break;
        }

        case CanConfig::Board::PAYLOAD:
        {
            status.payloadCounter = 4;
            status.payloadArmed   = armed;
            break;
        }

        case CanConfig::Board::MOTOR:
        {
            status.motorCounter = 4;
            break;
        }

        default:
        {
        }
    }
}

void CanHandler::periodicMessage()
{
    GroundModeManagerState state = getModule<GroundModeManager>()->getState();

    protocol.enqueueSimplePacket(
        static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
        static_cast<uint8_t>(CanConfig::PrimaryType::STATUS),
        static_cast<uint8_t>(CanConfig::Board::RIG),
        static_cast<uint8_t>(CanConfig::Board::BROADCAST), 0x00,
        (state == GroundModeManagerState::GMM_STATE_ARMED) ? 1 : 0);

    // Update status counters
    Lock<FastMutex> lock{statusMutex};
    status.mainCounter    = std::max(0, status.mainCounter - 1);
    status.payloadCounter = std::max(0, status.payloadCounter - 1);
    status.motorCounter   = std::max(0, status.motorCounter - 1);
}