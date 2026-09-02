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

#include "CanHandler.h"

#include <Motor/Actuators/Actuators.h>
#include <Motor/Configs/CanHandlerConfig.h>
#include <Motor/Sensors/Sensors.h>
#include <Motor/StateMachines/EregController/EregControllerFuel.h>
#include <Motor/StateMachines/EregController/EregControllerOx.h>
#include <Motor/StateMachines/FiringSequenceHSM/FiringSequenceHSM.h>
#include <Motor/StateMachines/MEAController/MEAController.h>
#include <common/CanConfig.h>
#include <common/canbus/MotorStatus.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <events/EventData.h>

using namespace miosix;
using namespace Motor;
using namespace Boardcore;
using namespace Canbus;
using namespace Common;

CanHandler::CanHandler()
    : driver(CAN1, CanConfig::CONFIG, CanConfig::BIT_TIMING),
      protocol(
          &driver, [this](const CanMessage& msg) { handleMessage(msg); },
          getModule<BoardScheduler>()->canHandlerPriority())
{
    protocol.addFilter(static_cast<uint8_t>(CanConfig::Board::RIG),
                       static_cast<uint8_t>(CanConfig::Board::BROADCAST));
    protocol.addFilter(static_cast<uint8_t>(CanConfig::Board::MAIN),
                       static_cast<uint8_t>(CanConfig::Board::BROADCAST));
}

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
            uint8_t state     = static_cast<uint8_t>(
                getModule<FiringSequenceHSM>()->getState());

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::STATUS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST), 0x00,
                DeviceStatus{
                    .timestamp = TimestampTimer::getTimestamp(),
                    .logNumber = static_cast<int16_t>(stats.logNumber),
                    .state     = state,
                    .armed     = false,
                    .hil       = PersistentVars::getHilMode(),
                    .logGood   = stats.lastWriteError == 0,
                });
        },
        CanConfig::STATUS_SEND_PERIOD);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to insert status update");
        return false;
    }

    result = scheduler.addTask(
        [this]()
        {
            Sensors* sensors = getModule<Sensors>();

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::OX_TANK_PRESSURE),
                static_cast<PressureData>(sensors->getOxTankPressure()));

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::MAIN_CC_PRESSURE),
                static_cast<PressureData>(sensors->getMainCCPressure()));
        },
        Config::CanHandler::CRITICAL_PRESSURE_SEND_RATE);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to insert pressure update");
        return false;
    }

    result = scheduler.addTask(
        [this]()
        {
            Sensors* sensors = getModule<Sensors>();

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::FUEL_TANK_PRESSURE),
                static_cast<PressureData>(sensors->getFuelTankPressure()));

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::PRZ_TANK_PRESSURE),
                static_cast<PressureData>(sensors->getPrzTankPressure()));

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::IGN_CC_PRESSURE),
                static_cast<PressureData>(
                    sensors->getIgniterChamberPressure()));
        },
        Config::CanHandler::SECONDARY_PRESSURE_SEND_RATE);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to insert pressure update");
        return false;
    }

    result = scheduler.addTask(
        [this]()
        {
            Sensors* sensors = getModule<Sensors>();

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::MOTOR_BOARD_VOLTAGE),
                static_cast<VoltageData>(sensors->getBatteryVoltage()));

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::MOTOR_BOARD_CURRENT),
                static_cast<CurrentData>(
                    sensors->getServoCurrentConsumption()));
        },
        Config::CanHandler::SENSORS_SEND_RATE);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to insert temperature update");
        return false;
    }

    result = scheduler.addTask(
        [this]()
        {
            Actuators* actuators = getModule<Actuators>();
            Sensors* sensors     = getModule<Sensors>();

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::ACTUATORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(0x0),
                ValveData{
                    TimestampTimer::getTimestamp(),
                    ServosList::OX_VENTING_VALVE,
                    static_cast<uint8_t>(
                        sensors->getVentingOxPosition().position),
                    actuators->isValveOpen(ServosList::OX_VENTING_VALVE)});

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::ACTUATORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(0x0),
                ValveData{
                    TimestampTimer::getTimestamp(),
                    ServosList::FUEL_VENTING_VALVE,
                    static_cast<uint8_t>(
                        sensors->getVentingFuelPosition().position),
                    actuators->isValveOpen(ServosList::FUEL_VENTING_VALVE)});

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::ACTUATORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(0x0),
                ValveData{
                    TimestampTimer::getTimestamp(), ServosList::MAIN_OX_VALVE,
                    static_cast<uint8_t>(sensors->getMainOxPosition().position),
                    actuators->isValveOpen(ServosList::MAIN_OX_VALVE)});

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::ACTUATORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(0x0),
                ValveData{TimestampTimer::getTimestamp(),
                          ServosList::MAIN_FUEL_VALVE,
                          static_cast<uint8_t>(
                              sensors->getMainFuelPosition().position),
                          actuators->isValveOpen(ServosList::MAIN_FUEL_VALVE)});

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::ACTUATORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(0x0),
                ValveData{
                    TimestampTimer::getTimestamp(), ServosList::PRZ_OX_VALVE,
                    static_cast<uint8_t>(sensors->getPrzOxPosition().position),
                    actuators->isValveOpen(ServosList::PRZ_OX_VALVE)});

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::ACTUATORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(0x0),
                ValveData{TimestampTimer::getTimestamp(),
                          ServosList::PRZ_FUEL_VALVE,
                          static_cast<uint8_t>(actuators->getValvePosition(
                              ServosList::PRZ_FUEL_VALVE)),
                          actuators->isValveOpen(ServosList::PRZ_FUEL_VALVE)});
        },
        Config::CanHandler::VALVE_STATE_SEND_RATE);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to insert actuators update");
        return false;
    }

    result = scheduler.addTask(
        [this]()
        {
            auto meaState = getModule<MEAController>()->getMEAState();

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::ALGORITHM),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::AlgoId::MEA_STATE),
                static_cast<MeaData>(MeaData{meaState.mass}));
        },
        Config::CanHandler::MEA_STATE_SEND_RATE);

    if (!protocol.start())
    {
        LOG_ERR(logger, "Failed to start CanProtocol");
        return false;
    }

    return true;
}

void CanHandler::handleMessage(const Canbus::CanMessage& msg)
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

        case CanConfig::PrimaryType::COMMAND:
        {
            handleCommand(msg);
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

void CanHandler::handleSensor(const Canbus::CanMessage& msg)
{
    auto sensor = static_cast<CanConfig::SensorId>(msg.getSecondaryType());
    LOG_WARN(logger, "Received unsupported sensor data: {}", sensor);
}

void CanHandler::handleCommand(const Canbus::CanMessage& msg)
{
    CanConfig::CommandId commandId =
        static_cast<CanConfig::CommandId>(msg.getSecondaryType());

    switch (commandId)
    {
        case Common::CanConfig::CommandId::SERVO_COMMAND:
        {
            CanServoCommand command = servoCommandFromCanMessage(msg);
            ServosList servoId      = static_cast<ServosList>(command.servoId);
            sdLogger.log(command);

            if (command.openingTime == 0)
                getModule<Actuators>()->closeValve(servoId);
            else
                getModule<Actuators>()->openValveWithTime(servoId,
                                                          command.openingTime);
            break;
        }
        case Common::CanConfig::CommandId::FIRING_SEQUENCE_CONFIG:
        {
            CanSequenceConfig config = sequenceConfigFromCanMessage(msg);
            sdLogger.log(config);

            getModule<FiringSequenceHSM>()->setFiringParams(
                config.fullThrottleTime, config.lowThrottleTime,
                config.pilotLeadTime, config.pilotOxPosition,
                config.pilotFuelPosition);
            break;
        }
        case Common::CanConfig::CommandId::EREG_TARGET:
        {
            CanEregTarget target = eregTargetFromCanMessage(msg);
            sdLogger.log(target);

            getModule<EregControllerOx>()->setEregTarget(target.oxTarget);
            getModule<EregControllerFuel>()->setEregTarget(target.fuelTarget);
            break;
        }
        case Common::CanConfig::CommandId::IGNITION_THRESHOLDS:
        {
            CanIgnitionThresholds thresholds =
                ignitionThresholdsFromCanMessage(msg);
            sdLogger.log(thresholds);

            getModule<FiringSequenceHSM>()->setPressureThresholds(
                thresholds.igniterThreshold, thresholds.pilotThreshold);
            break;
        }
        case Common::CanConfig::CommandId::EREG_PID_CONFIGS:
        {
            CanEregPIDSet configs = eregPIDSetFromCanMessage(msg);
            sdLogger.log(configs);

            if (configs.eregId == EregList::EREG_OX)
            {
                getModule<EregControllerOx>()->changePIDConfig(
                    {configs.KpPressurization, configs.KiPressurization,
                     configs.KdPressurization},
                    {configs.KpDischarge, configs.KiDischarge,
                     configs.KdDischarge});
            }
            else if (configs.eregId == EregList::EREG_FUEL)
            {
                getModule<EregControllerFuel>()->changePIDConfig(
                    {configs.KpPressurization, configs.KiPressurization,
                     configs.KdPressurization},
                    {configs.KpDischarge, configs.KiDischarge,
                     configs.KdDischarge});
            }
            break;
        }
        case Common::CanConfig::CommandId::EREG_SERVO_COEFFICIENTS:
        {
            CanEregServoCoefficients coeffs =
                canServoCoefficientsFromCanMessage(msg);
            sdLogger.log(coeffs);

            if (coeffs.eregId == EregList::EREG_OX)
            {
                getModule<EregControllerOx>()->setServoCoeff(
                    coeffs.coefficients);
            }
            else if (coeffs.eregId == EregList::EREG_FUEL)
            {
                getModule<EregControllerFuel>()->setServoCoeff(
                    coeffs.coefficients);
            }

            break;
        }
        default:
        {
            LOG_WARN(logger, "Received unsupported command: {}", commandId);
        }
    }
}

void CanHandler::handleEvent(const Canbus::CanMessage& msg)
{
    CanConfig::EventId event = static_cast<CanConfig::EventId>(
        msg.payload[0] & 0xFF);  // Extract the event ID from the payload

    switch (event)
    {
        case Common::CanConfig::EventId::ENTER_HIL_MODE:
        {
            PersistentVars::setHilMode(true);
            Thread::sleep(100);
            miosix::reboot();
            break;
        }
        case Common::CanConfig::EventId::EXIT_HIL_MODE:
        {
            if (PersistentVars::getHilMode())
            {
                PersistentVars::setHilMode(false);
                Thread::sleep(100);
                miosix::reboot();
            }
            break;
        }
        case Common::CanConfig::EventId::CLOSE_ALL_VALVES:
        {
            EventBroker::getInstance().post(CLOSE_ALL_VALVES,
                                            TOPIC_VALVE_SEQUENCE);
            break;
        }
        case Common::CanConfig::EventId::CALIBRATE:
        {
            getModule<Sensors>()->calibrate();
            getModule<Sensors>()->calibrateEncoders();
            EventBroker::getInstance().post(MEA_CALIBRATE, TOPIC_MEA);
            break;
        }
        case Common::CanConfig::EventId::APOGEE_DETECTED:
        {
            EventBroker::getInstance().post(CAN_APOGEE_DETECTED,
                                            TOPIC_FIRING_SEQUENCE);
            break;
        }
        case Common::CanConfig::EventId::ENTER_TEST_MODE:
        {
            EventBroker::getInstance().post(MEA_FORCE_START, TOPIC_MEA);
            break;
        }
        case Common::CanConfig::EventId::EXIT_TEST_MODE:
        {
            EventBroker::getInstance().post(MEA_FORCE_STOP, TOPIC_MEA);
            break;
        }
        case Common::CanConfig::EventId::EREG_OX_TOGGLE:
        {
            EventBroker::getInstance().post(EREG_TOGGLE, TOPIC_EREG_OX);
            break;
        }
        case Common::CanConfig::EventId::EREG_FUEL_TOGGLE:
        {
            EventBroker::getInstance().post(EREG_TOGGLE, TOPIC_EREG_FUEL);
            break;
        }
        case Common::CanConfig::EventId::IGNITION:
        {
            EventBroker::getInstance().post(FIRING_SEQUENCE_START,
                                            TOPIC_FIRING_SEQUENCE);

            break;
        }
        case Common::CanConfig::EventId::ENGINE_SHUTDOWN:
        {
            EventBroker::getInstance().post(FIRING_SEQUENCE_END,
                                            TOPIC_FIRING_SEQUENCE);
            break;
        }
        case Common::CanConfig::EventId::DISARM:
        {
            EventBroker::getInstance().post(FIRING_SEQUENCE_ABORT,
                                            TOPIC_FIRING_SEQUENCE);
            break;
        }

        default:
            LOG_WARN(logger, "Received unsupported event: {}", event);
            break;
    }

    // Log the event
    sdLogger.log(CanEvent{TimestampTimer::getTimestamp(), msg.getSource(),
                          msg.getDestination(),
                          static_cast<uint8_t>(msg.payload[0] & 0xFF)});
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

