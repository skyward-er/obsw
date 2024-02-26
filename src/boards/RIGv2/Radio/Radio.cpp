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

#include "Radio.h"

#include <RIGv2/Actuators/Actuators.h>
#include <RIGv2/Buses.h>
#include <RIGv2/Sensors/Sensors.h>
#include <common/Radio.h>
#include <events/EventBroker.h>
#include <radio/SX1278/SX1278Frontends.h>
// TODO(davide.mor): Remove TimestampTimer
#include <drivers/timer/TimestampTimer.h>

#include <atomic>

using namespace Boardcore;
using namespace miosix;
using namespace Common;
using namespace RIGv2;

SX1278Lora* gRadio{nullptr};

void handleDioIRQ()
{
    SX1278Lora* instance = gRadio;
    if (instance)
    {
        instance->handleDioIRQ();
    }
}

void setIRQRadio(SX1278Lora* radio)
{
    FastInterruptDisableLock dl;
    gRadio = radio;
}

void __attribute__((used)) MIOSIX_RADIO_DIO0_IRQ() { handleDioIRQ(); }
void __attribute__((used)) MIOSIX_RADIO_DIO1_IRQ() { handleDioIRQ(); }
void __attribute__((used)) MIOSIX_RADIO_DIO3_IRQ() { handleDioIRQ(); }

bool Radio::isStarted() { return started; }

bool Radio::start()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Setup the frontend
    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<EbyteFrontend>(radio::txEn::getPin(),
                                        radio::rxEn::getPin());

    // Setup transceiver
    radio = std::make_unique<SX1278Lora>(
        modules.get<Buses>()->getRadio(), radio::cs::getPin(),
        radio::dio0::getPin(), radio::dio1::getPin(), radio::dio3::getPin(),
        SPI::ClockDivider::DIV_64, std::move(frontend));

    // Store the global radio instance
    setIRQRadio(radio.get());

    // Initialize radio
    auto result = radio->init(RIG_RADIO_CONFIG);
    if (result != SX1278Lora::Error::NONE)
    {
        LOG_ERR(logger, "Failed to initialize RIG radio");
        return false;
    }

    // Initialize mavdriver
    mavDriver = std::make_unique<MavDriver>(
        radio.get(),
        [this](MavDriver*, const mavlink_message_t& msg)
        { handleMessage(msg); },
        Config::Radio::MAV_SLEEP_AFTER_SEND,
        Config::Radio::MAV_OUT_BUFFER_MAX_AGE);

    if (!mavDriver->start())
    {
        LOG_ERR(logger, "Failed to initialize RIG mav driver");
        return false;
    }

    started = true;
    return true;
}

void Radio::stop()
{
    // Remove global radio instance
    setIRQRadio(nullptr);

    mavDriver->stop();
    started = false;
}

void Radio::enqueuePacket(const mavlink_message_t& msg)
{
    queuedPackets.put(msg);
}

void Radio::flushPackets()
{
    // Flush all packets of the queue
    size_t count = queuedPackets.count();
    for (size_t i = 0; i < count; i++)
    {
        try
        {
            mavDriver->enqueueMsg(queuedPackets.pop());
        }
        catch (...)
        {
            // This shouldn't happen, but still try to prevent it
        }
    }
}

void Radio::sendAck(const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                            Config::Radio::MAV_COMPONENT_ID, &ackMsg, msg.msgid,
                            msg.seq);
    enqueuePacket(ackMsg);
}

void Radio::sendNack(const mavlink_message_t& msg)
{
    mavlink_message_t nackMsg;
    mavlink_msg_nack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                             Config::Radio::MAV_COMPONENT_ID, &nackMsg,
                             msg.msgid, msg.seq);
    enqueuePacket(nackMsg);
}

Boardcore::MavlinkStatus Radio::getMavStatus()
{
    return mavDriver->getStatus();
}

void Radio::handleMessage(const mavlink_message_t& msg)
{
    ModuleManager& modules = ModuleManager::getInstance();
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_PING_TC:
        {
            sendAck(msg);
            break;
        }

        case MAVLINK_MSG_ID_COMMAND_TC:
        {
            handleCommand(msg);
            break;
        }

        case MAVLINK_MSG_ID_CONRIG_STATE_TC:
        {
            handleConrigState(msg);
            break;
        }

        case MAVLINK_MSG_ID_SYSTEM_TM_REQUEST_TC:
        {
            uint8_t tmId = mavlink_msg_system_tm_request_tc_get_tm_id(&msg);

            mavlink_message_t tm;
            if (packSystemTm(tmId, tm))
            {
                sendAck(msg);
                enqueuePacket(tm);
            }
            else
            {
                sendNack(msg);
            }

            break;
        }

        case MAVLINK_MSG_ID_WIGGLE_SERVO_TC:
        {
            ServosList servo = static_cast<ServosList>(
                mavlink_msg_wiggle_servo_tc_get_servo_id(&msg));

            if (modules.get<Actuators>()->wiggleServo(servo))
            {
                sendAck(msg);
            }
            else
            {
                sendNack(msg);
            }
            break;
        }

        case MAVLINK_MSG_ID_SET_ATOMIC_VALVE_TIMING_TC:
        {
            uint32_t time =
                mavlink_msg_set_atomic_valve_timing_tc_get_maximum_timing(&msg);
            ServosList servo = static_cast<ServosList>(
                mavlink_msg_set_atomic_valve_timing_tc_get_servo_id(&msg));

            if (modules.get<Actuators>()->setOpeningTime(servo, time))
            {
                sendAck(msg);
            }
            else
            {
                sendNack(msg);
            }
            break;
        }

        case MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC:
        {
            float aperture =
                mavlink_msg_set_valve_maximum_aperture_tc_get_maximum_aperture(
                    &msg);
            ServosList servo = static_cast<ServosList>(
                mavlink_msg_set_valve_maximum_aperture_tc_get_servo_id(&msg));

            if (modules.get<Actuators>()->setMaxAperture(servo, aperture))
            {
                sendAck(msg);
            }
            else
            {
                sendNack(msg);
            }
            break;
        }

        case MAVLINK_MSG_ID_SET_IGNITION_TIME_TC:
        {
            // TODO(davide.mor): Implement set ignition time
            sendNack(msg);
            break;
        }

        default:
        {
            // Unrecognized packet
            sendNack(msg);
            break;
        }
    }
}

void Radio::handleCommand(const mavlink_message_t& msg)
{
    uint8_t cmd = mavlink_msg_command_tc_get_command_id(&msg);
    switch (cmd)
    {
        case MAV_CMD_START_LOGGING:
        {
            if (!Logger::getInstance().start())
            {
                sendNack(msg);
            }
            else
            {
                sendAck(msg);
            }
            break;
        }

        case MAV_CMD_STOP_LOGGING:
        {
            Logger::getInstance().stop();
            sendAck(msg);
            break;
        }

        case MAV_CMD_CALIBRATE:
        {
            // TODO: Implement calibrate
            sendNack(msg);
            break;
        }

        default:
        {
            // Unrecognized command
            sendNack(msg);
            break;
        }
    }
}

bool Radio::packSystemTm(uint8_t tmId, mavlink_message_t& msg)
{
    ModuleManager& modules = ModuleManager::getInstance();
    switch (tmId)
    {
        case MAV_SYS_ID:
        {
            mavlink_sys_tm_t tm;

            tm.timestamp    = TimestampTimer::getTimestamp();
            tm.logger       = Logger::getInstance().isStarted();
            tm.event_broker = EventBroker::getInstance().isRunning();
            // What? Why is this here? Of course the radio is started!
            tm.radio           = isStarted();
            tm.sensors         = modules.get<Sensors>()->isStarted();
            tm.board_scheduler = 0;  // TODO(davide.mor): No BoardScheduler yet

            mavlink_msg_sys_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            return true;
        }

        case MAV_LOGGER_ID:
        {
            mavlink_logger_tm_t tm;

            LoggerStats stats = Logger::getInstance().getStats();

            tm.timestamp          = TimestampTimer::getTimestamp();
            tm.log_number         = stats.logNumber;
            tm.too_large_samples  = stats.tooLargeSamples;
            tm.dropped_samples    = stats.droppedSamples;
            tm.queued_samples     = stats.queuedSamples;
            tm.buffers_filled     = stats.buffersFilled;
            tm.buffers_written    = stats.buffersWritten;
            tm.writes_failed      = stats.writesFailed;
            tm.last_write_error   = stats.lastWriteError;
            tm.average_write_time = stats.averageWriteTime;
            tm.max_write_time     = stats.maxWriteTime;

            mavlink_msg_logger_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                         Config::Radio::MAV_COMPONENT_ID, &msg,
                                         &tm);
            return true;
        }

        case MAV_MAVLINK_STATS:
        {
            mavlink_mavlink_stats_tm_t tm;

            MavlinkStatus stats = modules.get<Radio>()->mavDriver->getStatus();

            tm.timestamp               = stats.timestamp;
            tm.n_send_queue            = stats.nSendQueue;
            tm.max_send_queue          = stats.maxSendQueue;
            tm.n_send_errors           = stats.nSendErrors;
            tm.msg_received            = stats.mavStats.msg_received;
            tm.buffer_overrun          = stats.mavStats.buffer_overrun;
            tm.parse_error             = stats.mavStats.parse_error;
            tm.parse_state             = stats.mavStats.parse_state;
            tm.packet_idx              = stats.mavStats.packet_idx;
            tm.current_rx_seq          = stats.mavStats.current_rx_seq;
            tm.current_tx_seq          = stats.mavStats.current_tx_seq;
            tm.packet_rx_success_count = stats.mavStats.packet_rx_success_count;
            tm.packet_rx_drop_count    = stats.mavStats.packet_rx_drop_count;

            mavlink_msg_mavlink_stats_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                                Config::Radio::MAV_COMPONENT_ID,
                                                &msg, &tm);
            return true;
        }

        case MAV_GSE_ID:
        {
            mavlink_gse_tm_t tm = {0};

            tm.timestamp       = TimestampTimer::getTimestamp();
            tm.loadcell_rocket = 69;
            tm.loadcell_vessel = 420;
            // TODO(davide.mor): Add the rest of these

            mavlink_msg_gse_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            return true;
        }

        case MAV_MOTOR_ID:
        {
            mavlink_motor_tm_t tm;

            auto tc1 = modules.get<Sensors>()->getTc1LastSample();

            tm.timestamp            = TimestampTimer::getTimestamp();
            tm.tank_temperature     = tc1.temperature;
            tm.top_tank_pressure    = 69;
            tm.bottom_tank_pressure = 420;

            mavlink_msg_motor_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                        Config::Radio::MAV_COMPONENT_ID, &msg,
                                        &tm);
            return true;
        }

        default:
        {
            return false;
        }
    }
}

void Radio::handleConrigState(const mavlink_message_t& msg)
{
    // Acknowledge the state
    sendAck(msg);

    // Flush all pending packets
    flushPackets();

    // Send GSE and motor telemetry
    mavlink_message_t tm;
    packSystemTm(MAV_GSE_ID, tm);
    enqueuePacket(tm);
    packSystemTm(MAV_MOTOR_ID, tm);
    enqueuePacket(tm);

    // TODO: Handle buttons
}
