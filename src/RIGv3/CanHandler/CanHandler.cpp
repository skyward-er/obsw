/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Pietro Bortolus
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

#include <RIGv3/BoardScheduler.h>
#include <RIGv3/StateMachines/GroundModeManager/GroundModeManager.h>
#include <common/CanConfig.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <events/EventData.h>

using namespace miosix;
using namespace RIGv3;
using namespace Boardcore;
using namespace Canbus;
using namespace Common;

CanHandler::CanHandler()
    : driver(CAN1, CanConfig::CONFIG, CanConfig::BIT_TIMING),
      protocol(
          &driver, [this](const CanMessage& msg) { handleMessage(msg); },
          BoardScheduler::canHandlerPriority())
{
    protocol.addFilter(static_cast<uint8_t>(CanConfig::Board::MAIN),
                       static_cast<uint8_t>(CanConfig::Board::BROADCAST));
    protocol.addFilter(static_cast<uint8_t>(CanConfig::Board::MOTOR),
                       static_cast<uint8_t>(CanConfig::Board::BROADCAST));
}

bool CanHandler::isStarted() { return started; }

bool CanHandler::start()
{
    if (!driver.init(Common::CanConfig::CAN_SYNC_TIMEOUT))
    {
        LOG_ERR(logger, "Failed to initialize CanbusDriver");
        return false;
    }

    TaskScheduler& scheduler = getModule<BoardScheduler>()->canHandler();

    uint8_t result = scheduler.addTask(
        [this]()
        {
            LoggerStats stats = sdLogger.getStats();

            GroundModeManagerState state =
                getModule<GroundModeManager>()->getState();

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::STATUS),
                static_cast<uint8_t>(CanConfig::Board::RIG),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST), 0x00,
                DeviceStatus{
                    TimestampTimer::getTimestamp(),
                    static_cast<int16_t>(stats.logNumber),
                    static_cast<uint8_t>(state),
                    state == GroundModeManagerState::ARMED,
                    false,
                    stats.lastWriteError == 0,
                });
        },
        Common::CanConfig::STATUS_SEND_PERIOD);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add periodicMessageTask");
        return false;
    }

    if (!protocol.start())
    {
        LOG_ERR(logger, "Failed to start CanProtocol");
        return false;
    }

    started = true;
    return true;
}

void CanHandler::sendEvent(CanConfig::EventId event)
{
    sdLogger.log(CanEvent{TimestampTimer::getTimestamp(),
                          static_cast<uint8_t>(CanConfig::Board::RIG),
                          static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                          static_cast<uint8_t>(event)});

    protocol.enqueueEvent(static_cast<uint8_t>(CanConfig::Priority::CRITICAL),
                          static_cast<uint8_t>(CanConfig::PrimaryType::EVENTS),
                          static_cast<uint8_t>(CanConfig::Board::RIG),
                          static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                          static_cast<uint8_t>(0x0),
                          static_cast<uint8_t>(event));
}

void CanHandler::sendServoOpenCommand(ServosList servo, uint32_t openingTime)
{
    protocol.enqueueData(
        static_cast<uint8_t>(CanConfig::Priority::CRITICAL),
        static_cast<uint8_t>(CanConfig::PrimaryType::COMMAND),
        static_cast<uint8_t>(CanConfig::Board::RIG),
        static_cast<uint8_t>(CanConfig::Board::BROADCAST),
        static_cast<uint8_t>(CanConfig::CommandId::SERVO_COMMAND),
        ServoCommand{TimestampTimer::getTimestamp(), openingTime,
                     static_cast<uint8_t>(servo)});
}

void CanHandler::sendServoCloseCommand(ServosList servo)
{
    // Closing a servo means opening it for 0s
    sendServoOpenCommand(servo, 0);
}

void CanHandler::sendIgnitionSequenceConfig(uint32_t fullThrottleTime,
                                            uint32_t lowThrottleTime,
                                            uint32_t pilotLeadTime,
                                            float pilotOxPosition,
                                            float pilotFuelPosition)
{
    protocol.enqueueData(
        static_cast<uint8_t>(CanConfig::Priority::HIGH),
        static_cast<uint8_t>(CanConfig::PrimaryType::COMMAND),
        static_cast<uint8_t>(CanConfig::Board::RIG),
        static_cast<uint8_t>(CanConfig::Board::BROADCAST),
        static_cast<uint8_t>(CanConfig::CommandId::FIRING_SEQUENCE_CONFIG),
        SequenceConfig{TimestampTimer::getTimestamp(), fullThrottleTime,
                       lowThrottleTime, pilotLeadTime, pilotOxPosition,
                       pilotFuelPosition});
}

void CanHandler::sendEregPIDConfigs(EregPIDConfig pressurizationConfig,
                                    EregPIDConfig dischargeConfig,
                                    EregList eregId)
{
    protocol.enqueueData(
        static_cast<uint8_t>(CanConfig::Priority::HIGH),
        static_cast<uint8_t>(CanConfig::PrimaryType::COMMAND),
        static_cast<uint8_t>(CanConfig::Board::RIG),
        static_cast<uint8_t>(CanConfig::Board::BROADCAST),
        static_cast<uint8_t>(CanConfig::CommandId::EREG_PID_CONFIGS),
        EregPIDSet{pressurizationConfig.KP, pressurizationConfig.KI,
                   pressurizationConfig.KD, dischargeConfig.KP,
                   dischargeConfig.KI, dischargeConfig.KD,
                   static_cast<uint8_t>(eregId)});
}

void CanHandler::sendEregTarget(float oxTarget, float fuelTarget)
{
    protocol.enqueueData(
        static_cast<uint8_t>(CanConfig::Priority::HIGH),
        static_cast<uint8_t>(CanConfig::PrimaryType::COMMAND),
        static_cast<uint8_t>(CanConfig::Board::RIG),
        static_cast<uint8_t>(CanConfig::Board::BROADCAST),
        static_cast<uint8_t>(CanConfig::CommandId::EREG_TARGET),
        EregTarget{oxTarget, fuelTarget});
}

void CanHandler::sendIgnitionThresholds(float igniterThreshold,
                                        float pilotThreshold)
{
    protocol.enqueueData(
        static_cast<uint8_t>(CanConfig::Priority::HIGH),
        static_cast<uint8_t>(CanConfig::PrimaryType::COMMAND),
        static_cast<uint8_t>(CanConfig::Board::RIG),
        static_cast<uint8_t>(CanConfig::Board::BROADCAST),
        static_cast<uint8_t>(CanConfig::CommandId::IGNITION_THRESHOLDS),
        IgnitionThresholds{TimestampTimer::getTimestamp(), igniterThreshold,
                           pilotThreshold});

    sdLogger.log(IgnitionThresholds{TimestampTimer::getTimestamp(),
                                    igniterThreshold, pilotThreshold});
}

void CanHandler::sendEregServoCoeff(EregList eregId, float coefficients[])
{
    uint8_t id = static_cast<uint8_t>(eregId);

    protocol.enqueueData(
        static_cast<uint8_t>(CanConfig::Priority::HIGH),
        static_cast<uint8_t>(CanConfig::PrimaryType::COMMAND),
        static_cast<uint8_t>(CanConfig::Board::RIG),
        static_cast<uint8_t>(CanConfig::Board::BROADCAST),
        static_cast<uint8_t>(CanConfig::CommandId::EREG_SERVO_COEFFICIENTS),
        EregServoCoefficients{id, coefficients});

    sdLogger.log(EregServoCoefficients{id, coefficients});
}

CanHandler::CanStatus CanHandler::getCanStatus()
{
    Lock<FastMutex> lock{statusMutex};
    return status;
}

void CanHandler::handleMessage(const Canbus::CanMessage& msg)
{
    // Handle motor messages
    auto source = static_cast<CanConfig::Board>(msg.getSource());
    if (source == CanConfig::Board::MOTOR)

        return getModule<MotorStatus>()->handleCanMessage(msg);

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

void CanHandler::handleEvent(const Canbus::CanMessage& msg)
{
    sdLogger.log(CanEvent{TimestampTimer::getTimestamp(), msg.getSource(),
                          msg.getDestination(), msg.getSecondaryType()});

    // Dispatch the event, so that the logger can log it
    Events event = canEventToEvent(msg.getSecondaryType());
    if (event != LAST_EVENT)
    {
        EventBroker::getInstance().post(event, TOPIC_CAN);
    }
    else
    {
        LOG_WARN(logger, "Received unsupported event: {}",
                 msg.getSecondaryType());
    }
}

void CanHandler::handleSensor(const Canbus::CanMessage& msg)
{
    CanConfig::SensorId sensor =
        static_cast<CanConfig::SensorId>(msg.getSecondaryType());

    switch (sensor)
    {
        default:
        {
            LOG_WARN(logger, "Received unsupported sensor data: {}", sensor);
        }
    }
}

void CanHandler::handleActuator(const Canbus::CanMessage& msg)
{
    CanValveData data = valveDataFromCanMessage(msg);
    sdLogger.log(data);
}

void CanHandler::handleStatus(const Canbus::CanMessage& msg)
{
    CanConfig::Board source = static_cast<CanConfig::Board>(msg.getSource());
    CanDeviceStatus deviceStatus = deviceStatusFromCanMessage(msg);

    Lock<FastMutex> lock{statusMutex};

    switch (source)
    {
        case CanConfig::Board::MAIN:
        {
            status.mainLastStatus = getTime();
            status.mainArmed      = deviceStatus.armed;
            status.mainState      = deviceStatus.state;
            break;
        }

        default:
        {
            LOG_WARN(logger, "Received unsupported status: {}", source);
        }
    }
}
