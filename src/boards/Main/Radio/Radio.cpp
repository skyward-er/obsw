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

#include "Radio.h"

#include <Main/Buses.h>
#include <Main/Sensors/Sensors.h>
#include <common/Events.h>
#include <common/Radio.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <radio/SX1278/SX1278Frontends.h>

#include <map>

using namespace Main;
using namespace Boardcore;
using namespace miosix;
using namespace Common;

SX1278Fsk* gRadio{nullptr};

void handleDioIRQ()
{
    SX1278Fsk* instance = gRadio;
    if (instance)
    {
        instance->handleDioIRQ();
    }
}

void setIRQRadio(SX1278Fsk* radio)
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
        std::make_unique<Skyward433Frontend>();

    // Setup transceiver
    radio = std::make_unique<SX1278Fsk>(
        modules.get<Buses>()->getRadio(), radio::cs::getPin(),
        radio::dio0::getPin(), radio::dio1::getPin(), radio::dio3::getPin(),
        SPI::ClockDivider::DIV_64, std::move(frontend));

    // Store the global radio instance
    setIRQRadio(radio.get());

    // Initialize radio
    auto result = radio->init(MAIN_RADIO_CONFIG);
    if (result != SX1278Fsk::Error::NONE)
    {
        LOG_ERR(logger, "Failed to initialize Main radio");
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
        LOG_ERR(logger, "Failed to initialize Main mav driver");
        return false;
    }

    // High rate periodic telemetry
    if (scheduler.addTask(
            [this]()
            {
                enqueueSystemTm(SystemTMList::MAV_FLIGHT_ID);
                flushPackets();
            },
            Config::Radio::TELEMETRY_PERIOD) == 0)
    {
        LOG_ERR(logger, "Failed to add periodic telemetry task");
        return false;
    }

    if (scheduler.addTask([this]()
                          { enqueueSystemTm(SystemTMList::MAV_STATS_ID); },
                          Config::Radio::TELEMETRY_PERIOD * 2) == 0)
    {
        LOG_ERR(logger, "Failed to add periodic telemetry task");
        return false;
    }

    started = true;
    return true;
}

Boardcore::MavlinkStatus Radio::getMavStatus()
{
    return mavDriver->getStatus();
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

void Radio::enqueueAck(const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                            Config::Radio::MAV_COMPONENT_ID, &ackMsg, msg.msgid,
                            msg.seq);
    enqueuePacket(ackMsg);
}

void Radio::enqueueNack(const mavlink_message_t& msg)
{
    mavlink_message_t nackMsg;
    mavlink_msg_nack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                             Config::Radio::MAV_COMPONENT_ID, &nackMsg,
                             msg.msgid, msg.seq);
    enqueuePacket(nackMsg);
}

void Radio::handleMessage(const mavlink_message_t& msg)
{
    switch (msg.msgid)
    {

        case MAVLINK_MSG_ID_PING_TC:
        {
            enqueueAck(msg);
            break;
        }

        case MAVLINK_MSG_ID_COMMAND_TC:
        {
            handleCommand(msg);
            break;
        }

        case MAVLINK_MSG_ID_SYSTEM_TM_REQUEST_TC:
        {
            uint8_t tmId = mavlink_msg_system_tm_request_tc_get_tm_id(&msg);
            if (enqueueSystemTm(tmId))
            {
                enqueueAck(msg);
            }
            else
            {
                enqueueNack(msg);
            }

            break;
        }

        case MAVLINK_MSG_ID_SENSOR_TM_REQUEST_TC:
        {
            uint8_t tmId =
                mavlink_msg_sensor_tm_request_tc_get_sensor_name(&msg);
            if (enqueueSensorsTm(tmId))
            {
                enqueueAck(msg);
            }
            else
            {
                enqueueNack(msg);
            }

            break;
        }

        default:
        {
            enqueueNack(msg);
        }
    }
}

void Radio::handleCommand(const mavlink_message_t& msg)
{
    static const std::map<uint8_t, Events> cmdToEvent{
        {MAV_CMD_ARM, TMTC_ARM},
        {MAV_CMD_DISARM, TMTC_DISARM},
        {MAV_CMD_CALIBRATE, TMTC_CALIBRATE},
        {MAV_CMD_FORCE_INIT, TMTC_FORCE_INIT},
        {MAV_CMD_FORCE_LAUNCH, TMTC_FORCE_LAUNCH},
        {MAV_CMD_FORCE_LANDING, TMTC_FORCE_LANDING},
        {MAV_CMD_FORCE_APOGEE, TMTC_FORCE_APOGEE},
        {MAV_CMD_FORCE_EXPULSION, TMTC_FORCE_EXPULSION},
        {MAV_CMD_FORCE_DEPLOYMENT, TMTC_FORCE_DEPLOYMENT},
        {MAV_CMD_FORCE_REBOOT, TMTC_RESET_BOARD},
        {MAV_CMD_ENTER_TEST_MODE, TMTC_ENTER_TEST_MODE},
        {MAV_CMD_EXIT_TEST_MODE, TMTC_EXIT_TEST_MODE},
        {MAV_CMD_START_RECORDING, TMTC_START_RECORDING},
        {MAV_CMD_STOP_RECORDING, TMTC_STOP_RECORDING},
    };

    uint8_t cmdId = mavlink_msg_command_tc_get_command_id(&msg);
    switch (cmdId)
    {
        case MAV_CMD_START_LOGGING:
        {
            if (Logger::getInstance().start())
            {
                enqueueAck(msg);
            }
            else
            {
                enqueueNack(msg);
            }
            break;
        }

        case MAV_CMD_STOP_LOGGING:
        {
            Logger::getInstance().stop();
            break;
        }

        default:
        {
            // Try to map the command to an event
            auto it = cmdToEvent.find(cmdId);
            if (it != cmdToEvent.end())
            {
                EventBroker::getInstance().post(it->second, TOPIC_TMTC);
                enqueueAck(msg);
            }
            else
            {
                enqueueNack(msg);
            }
        }
    }
}

bool Radio::enqueueSystemTm(uint8_t tmId)
{
    ModuleManager& modules = ModuleManager::getInstance();

    switch (tmId)
    {
        case MAV_SENSORS_STATE_ID:
        {
            auto sensors = modules.get<Sensors>()->getSensorInfos();
            for (auto sensor : sensors)
            {
                mavlink_message_t msg;
                mavlink_sensor_state_tm_t tm;

                strcpy(tm.sensor_name, sensor.id.c_str());
                tm.state =
                    (sensor.isInitialized ? 1 : 0) | (sensor.isEnabled ? 2 : 0);

                mavlink_msg_sensor_state_tm_encode(
                    Config::Radio::MAV_SYSTEM_ID,
                    Config::Radio::MAV_COMPONENT_ID, &msg, &tm);
                enqueuePacket(msg);
            }

            return true;
        }

        case MAV_LOGGER_ID:
        {
            mavlink_message_t msg;
            mavlink_logger_tm_t tm;

            // Get the logger stats
            LoggerStats stats = Logger::getInstance().getStats();

            tm.timestamp          = stats.timestamp;
            tm.average_write_time = stats.averageWriteTime;
            tm.buffers_filled     = stats.buffersFilled;
            tm.buffers_written    = stats.buffersWritten;
            tm.dropped_samples    = stats.droppedSamples;
            tm.last_write_error   = stats.lastWriteError;
            tm.log_number         = stats.logNumber;
            tm.max_write_time     = stats.maxWriteTime;
            tm.queued_samples     = stats.queuedSamples;
            tm.too_large_samples  = stats.tooLargeSamples;
            tm.writes_failed      = stats.writesFailed;

            mavlink_msg_logger_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                         Config::Radio::MAV_COMPONENT_ID, &msg,
                                         &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_MAVLINK_STATS:
        {
            mavlink_message_t msg;
            mavlink_mavlink_stats_tm_t tm;

            // Get the mavlink stats
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
            enqueuePacket(msg);
            return true;
        }

        case MAV_FLIGHT_ID:
        {
            mavlink_message_t msg;
            mavlink_rocket_flight_tm_t tm;

            Sensors* sensors = modules.get<Sensors>();
            auto pressDigi   = sensors->getLPS22DFLastSample();
            auto imu         = sensors->getLSM6DSRXLastSample();
            auto mag         = sensors->getLIS2MDLLastSample();
            auto gps         = sensors->getUBXGPSLastSample();

            tm.timestamp           = TimestampTimer::getTimestamp();
            tm.pressure_digi       = pressDigi.pressure;
            tm.acc_x               = imu.accelerationX;
            tm.acc_y               = imu.accelerationY;
            tm.acc_z               = imu.accelerationZ;
            tm.gyro_x              = imu.angularSpeedX;
            tm.gyro_y              = imu.angularSpeedY;
            tm.gyro_z              = imu.angularSpeedZ;
            tm.mag_x               = mag.magneticFieldX;
            tm.mag_y               = mag.magneticFieldY;
            tm.mag_z               = mag.magneticFieldZ;
            tm.gps_alt             = gps.height;
            tm.gps_lat             = gps.latitude;
            tm.gps_lon             = gps.longitude;
            tm.gps_fix             = gps.fix;
            tm.battery_voltage     = sensors->getBatteryVoltage().voltage;
            tm.cam_battery_voltage = sensors->getCamBatteryVoltage().voltage;
            tm.logger_error = Logger::getInstance().getStats().lastWriteError;
            tm.temperature  = pressDigi.temperature;

            mavlink_msg_rocket_flight_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                                Config::Radio::MAV_COMPONENT_ID,
                                                &msg, &tm);
            enqueuePacket(msg);
            return true;
        }
        case MAV_STATS_ID:
        {
            mavlink_message_t msg;
            mavlink_rocket_stats_tm_t tm;

            mavlink_msg_rocket_stats_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                               Config::Radio::MAV_COMPONENT_ID,
                                               &msg, &tm);
            enqueuePacket(msg);
            return true;
        }
        default:
            return false;
    }
}

bool Radio::enqueueSensorsTm(uint8_t tmId)
{
    ModuleManager& modules = ModuleManager::getInstance();

    switch (tmId)
    {
        case MAV_GPS_ID:
        {
            mavlink_message_t msg;
            mavlink_gps_tm_t tm;

            auto sample = modules.get<Sensors>()->getUBXGPSLastSample();

            tm.fix          = sample.fix;
            tm.height       = sample.height;
            tm.latitude     = sample.latitude;
            tm.longitude    = sample.longitude;
            tm.n_satellites = sample.satellites;
            tm.speed        = sample.speed;
            tm.timestamp    = sample.gpsTimestamp;
            tm.track        = sample.track;
            tm.vel_down     = sample.velocityDown;
            tm.vel_east     = sample.velocityEast;
            tm.vel_north    = sample.velocityNorth;
            strcpy(tm.sensor_name, "UBXGPS");

            mavlink_msg_gps_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_ADS_ID:
        {
            mavlink_message_t msg;
            mavlink_adc_tm_t tm;

            auto sample = modules.get<Sensors>()->getADS131M08LastSample();

            tm.channel_0 =
                sample.getVoltage(ADS131M08Defs::Channel::CHANNEL_0).voltage;
            tm.channel_1 =
                sample.getVoltage(ADS131M08Defs::Channel::CHANNEL_1).voltage;
            tm.channel_2 =
                sample.getVoltage(ADS131M08Defs::Channel::CHANNEL_2).voltage;
            tm.channel_3 =
                sample.getVoltage(ADS131M08Defs::Channel::CHANNEL_3).voltage;
            tm.channel_4 =
                sample.getVoltage(ADS131M08Defs::Channel::CHANNEL_4).voltage;
            tm.channel_5 =
                sample.getVoltage(ADS131M08Defs::Channel::CHANNEL_5).voltage;
            tm.channel_6 =
                sample.getVoltage(ADS131M08Defs::Channel::CHANNEL_6).voltage;
            tm.channel_7 =
                sample.getVoltage(ADS131M08Defs::Channel::CHANNEL_7).voltage;
            tm.timestamp = sample.timestamp;
            strcpy(tm.sensor_name, "ADS131M08");

            mavlink_msg_adc_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_BATTERY_VOLTAGE_ID:
        {
            mavlink_message_t msg;
            mavlink_voltage_tm_t tm;

            auto data = modules.get<Sensors>()->getBatteryVoltage();

            tm.voltage   = data.voltage;
            tm.timestamp = data.voltageTimestamp;
            strcpy(tm.sensor_name, "BATTERY_VOLTAGE");

            mavlink_msg_voltage_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                          Config::Radio::MAV_COMPONENT_ID, &msg,
                                          &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_LPS28DFW_ID:
        {
            mavlink_message_t msg;
            mavlink_pressure_tm_t tm;

            auto sample = modules.get<Sensors>()->getLPS28DFWLastSample();

            tm.pressure  = sample.pressure;
            tm.timestamp = sample.pressureTimestamp;
            strcpy(tm.sensor_name, "LPS28DFW");

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm);

            enqueuePacket(msg);
            return true;
        }

        case MAV_LPS22DF_ID:
        {
            mavlink_message_t msg;
            mavlink_pressure_tm_t tm;

            auto sample = modules.get<Sensors>()->getLPS22DFLastSample();

            tm.pressure  = sample.pressure;
            tm.timestamp = sample.pressureTimestamp;
            strcpy(tm.sensor_name, "LPS22DF");

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm);

            enqueuePacket(msg);
            return true;
        }

        case MAV_LIS2MDL_ID:
        {
            mavlink_message_t msg;
            mavlink_imu_tm_t tm;

            auto sample = modules.get<Sensors>()->getLIS2MDLLastSample();

            tm.acc_x     = 0;
            tm.acc_y     = 0;
            tm.acc_z     = 0;
            tm.gyro_x    = 0;
            tm.gyro_y    = 0;
            tm.gyro_z    = 0;
            tm.mag_x     = sample.magneticFieldX;
            tm.mag_y     = sample.magneticFieldY;
            tm.mag_z     = sample.magneticFieldZ;
            tm.timestamp = sample.magneticFieldTimestamp;
            strcpy(tm.sensor_name, "LIS2MDL");

            mavlink_msg_imu_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_LSM6DSRX_ID:
        {
            mavlink_message_t msg;
            mavlink_imu_tm_t tm;

            auto sample = modules.get<Sensors>()->getLSM6DSRXLastSample();

            tm.mag_x     = 0;
            tm.mag_y     = 0;
            tm.mag_z     = 0;
            tm.acc_x     = sample.accelerationX;
            tm.acc_y     = sample.accelerationY;
            tm.acc_z     = sample.accelerationZ;
            tm.gyro_x    = sample.angularSpeedX;
            tm.gyro_y    = sample.angularSpeedY;
            tm.gyro_z    = sample.angularSpeedZ;
            tm.timestamp = sample.accelerationTimestamp;
            strcpy(tm.sensor_name, "LSM6DSRX");

            mavlink_msg_imu_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            enqueuePacket(msg);
            return true;
        }
        
        case MAV_H3LIS331DL_ID:
        {
            mavlink_message_t msg;
            mavlink_imu_tm_t tm;

            auto sample = modules.get<Sensors>()->getH3LIS331DLLastSample();

            tm.mag_x     = 0;
            tm.mag_y     = 0;
            tm.mag_z     = 0;
            tm.gyro_x    = 0;
            tm.gyro_y    = 0;
            tm.gyro_z    = 0;
            tm.acc_x     = sample.accelerationX;
            tm.acc_y     = sample.accelerationY;
            tm.acc_z     = sample.accelerationZ;
            tm.timestamp = sample.accelerationTimestamp;
            strcpy(tm.sensor_name, "H3LIS331DL");

            mavlink_msg_imu_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            enqueuePacket(msg);
            return true;
        }

        default:
            return false;
    }
}