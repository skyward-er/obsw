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

#include <Payload/Actuators/Actuators.h>
#include <Payload/AltitudeTrigger/AltitudeTrigger.h>
#include <Payload/BoardScheduler.h>
#include <Payload/CanHandler/CanHandler.h>
#include <Payload/PinHandler/PinHandler.h>
#include <Payload/Sensors/Sensors.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Payload/StateMachines/NASController/NASController.h>
#include <Payload/StateMachines/WingController/WingController.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <events/EventBroker.h>

#include "Radio.h"

using namespace Boardcore;
using namespace Common;
namespace config = Payload::Config::Radio;

namespace Payload
{

void Radio::handleMessage(const mavlink_message_t& msg)
{
    switch (msg.msgid)
    {

        case MAVLINK_MSG_ID_PING_TC:
        {
            return enqueueAck(msg);
        }

        case MAVLINK_MSG_ID_COMMAND_TC:
        {
            return handleCommand(msg);
        }

        case MAVLINK_MSG_ID_SYSTEM_TM_REQUEST_TC:
        {
            auto tmId = static_cast<SystemTMList>(
                mavlink_msg_system_tm_request_tc_get_tm_id(&msg));

            if (!enqueueSystemTm(tmId))
            {
                return enqueueNack(msg);
            }

            return enqueueAck(msg);
        }

        case MAVLINK_MSG_ID_SENSOR_TM_REQUEST_TC:
        {
            auto sensorId = static_cast<SensorsTMList>(
                mavlink_msg_sensor_tm_request_tc_get_sensor_name(&msg));

            if (!enqueueSensorsTm(sensorId))
            {
                return enqueueNack(msg);
            }

            return enqueueAck(msg);
        }

        case MAVLINK_MSG_ID_WIGGLE_SERVO_TC:
        {
            bool testMode = getModule<FlightModeManager>()->isTestMode();
            // Perform the wiggle in test mode only
            if (!testMode)
            {
                return enqueueNack(msg);
            }

            auto servo = static_cast<ServosList>(
                mavlink_msg_wiggle_servo_tc_get_servo_id(&msg));

            getModule<Actuators>()->wiggleServo(servo);
            return enqueueAck(msg);
        }

        case MAVLINK_MSG_ID_RAW_EVENT_TC:
        {
            uint8_t topicId = mavlink_msg_raw_event_tc_get_topic_id(&msg);
            uint8_t eventId = mavlink_msg_raw_event_tc_get_event_id(&msg);

            bool testMode = getModule<FlightModeManager>()->isTestMode();
            // Raw events are allowed in test mode only
            if (!testMode)
            {
                return enqueueNack(msg);
            }

            EventBroker::getInstance().post(topicId, eventId);
            return enqueueAck(msg);
        }

        default:
        {
            return enqueueNack(msg);
        }
    }
}

void Radio::handleCommand(const mavlink_message_t& msg)
{
    // Command-to-event look-up table, uses operator[] to retrieve an event from
    // a command
    static struct
    {
        Events operator[](MavCommandList command)
        {
#define MAP_CMD(command, event) \
    case command:               \
        return event

            switch (command)
            {
                MAP_CMD(MAV_CMD_ARM, TMTC_ARM);
                MAP_CMD(MAV_CMD_DISARM, TMTC_DISARM);
                MAP_CMD(MAV_CMD_CALIBRATE, TMTC_CALIBRATE);
                MAP_CMD(MAV_CMD_FORCE_INIT, TMTC_FORCE_INIT);
                MAP_CMD(MAV_CMD_FORCE_LAUNCH, TMTC_FORCE_LAUNCH);
                MAP_CMD(MAV_CMD_FORCE_LANDING, TMTC_FORCE_LANDING);
                MAP_CMD(MAV_CMD_FORCE_EXPULSION, TMTC_FORCE_EXPULSION);
                MAP_CMD(MAV_CMD_FORCE_DEPLOYMENT, TMTC_FORCE_DEPLOYMENT);
                MAP_CMD(MAV_CMD_FORCE_REBOOT, TMTC_RESET_BOARD);
                MAP_CMD(MAV_CMD_ENTER_TEST_MODE, TMTC_ENTER_TEST_MODE);
                MAP_CMD(MAV_CMD_EXIT_TEST_MODE, TMTC_EXIT_TEST_MODE);
                MAP_CMD(MAV_CMD_START_RECORDING, TMTC_START_RECORDING);
                MAP_CMD(MAV_CMD_STOP_RECORDING, TMTC_STOP_RECORDING);
                default:
                    return LAST_EVENT;
            }
#undef MAP_CMD
        }
    } commandToEvent;  // Command-to-event look-up table

    auto command = static_cast<MavCommandList>(
        mavlink_msg_command_tc_get_command_id(&msg));

    switch (command)
    {
        case MAV_CMD_START_LOGGING:
        {
            bool started = Logger::getInstance().start();
            if (!started)
            {
                return enqueueNack(msg);
            }

            Logger::getInstance().resetStats();
            return enqueueAck(msg);
        }

        case MAV_CMD_STOP_LOGGING:
        {
            Logger::getInstance().stop();
            return enqueueAck(msg);
        }

        case MAV_CMD_SAVE_CALIBRATION:
        {
            bool magResult = getModule<Sensors>()->writeMagCalibration();
            if (magResult)
            {
                return enqueueAck(msg);
            }
            else
            {
                return enqueueNack(msg);
            }
        }

        default:
        {
            // Map the command to an event and post it, if it exists
            auto event = commandToEvent[command];

            if (event == LAST_EVENT)
            {
                return enqueueNack(msg);
            }

            EventBroker::getInstance().post(event, TOPIC_TMTC);
            return enqueueAck(msg);
        }
    }
}

bool Radio::enqueueSystemTm(SystemTMList tmId)
{
    switch (tmId)
    {
        case MAV_SENSORS_STATE_ID:
        {
            auto sensors = getModule<Sensors>()->getSensorInfo();
            for (auto sensor : sensors)
            {
                mavlink_message_t msg;
                mavlink_sensor_state_tm_t tm;

                strcpy(tm.sensor_name, sensor.id.c_str());
                tm.initialized = sensor.isInitialized ? 1 : 0;
                tm.enabled     = sensor.isEnabled ? 1 : 0;

                mavlink_msg_sensor_state_tm_encode(
                    config::Mavlink::SYSTEM_ID, config::Mavlink::COMPONENT_ID,
                    &msg, &tm);
                enqueueMessage(msg);
            }

            return true;
        }

        case MAV_SYS_ID:
        {
            mavlink_message_t msg;
            mavlink_sys_tm_t tm;

            tm.timestamp    = TimestampTimer::getTimestamp();
            tm.logger       = Logger::getInstance().isStarted() ? 1 : 0;
            tm.event_broker = EventBroker::getInstance().isRunning() ? 1 : 0;
            tm.radio        = isStarted() ? 1 : 0;
            tm.sensors      = getModule<Sensors>()->isStarted() ? 1 : 0;
            tm.actuators    = getModule<Actuators>()->isStarted() ? 1 : 0;
            tm.pin_handler  = getModule<PinHandler>()->isStarted() ? 1 : 0;
            tm.can_handler  = getModule<CanHandler>()->isStarted() ? 1 : 0;
            tm.scheduler    = getModule<BoardScheduler>()->isStarted() ? 1 : 0;

            mavlink_msg_sys_tm_encode(config::Mavlink::SYSTEM_ID,
                                      config::Mavlink::COMPONENT_ID, &msg, &tm);
            enqueueMessage(msg);
            return true;
        }

        case MAV_LOGGER_ID:
        {
            mavlink_message_t msg;
            mavlink_logger_tm_t tm;

            // Get the logger stats
            LoggerStats stats = Logger::getInstance().getStats();

            tm.timestamp          = stats.timestamp;
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

            mavlink_msg_logger_tm_encode(config::Mavlink::SYSTEM_ID,
                                         config::Mavlink::COMPONENT_ID, &msg,
                                         &tm);
            enqueueMessage(msg);
            return true;
        }

        case MAV_MAVLINK_STATS_ID:
        {
            mavlink_message_t msg;
            mavlink_mavlink_stats_tm_t tm;

            // Get the mavlink stats
            MavlinkStatus stats = mavDriver->getStatus();

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

            mavlink_msg_mavlink_stats_tm_encode(config::Mavlink::SYSTEM_ID,
                                                config::Mavlink::COMPONENT_ID,
                                                &msg, &tm);
            enqueueMessage(msg);
            return true;
        }

        case MAV_FLIGHT_ID:
        {
            mavlink_message_t msg;
            mavlink_payload_flight_tm_t tm;

            Sensors* sensors       = getModule<Sensors>();
            PinHandler* pinHandler = getModule<PinHandler>();

            auto imu         = sensors->getLSM6DSRXLastSample();
            auto mag         = sensors->getLIS2MDLLastSample();
            auto gps         = sensors->getUBXGPSLastSample();
            auto pressDigi   = sensors->getLPS22DFLastSample();
            auto pressStatic = sensors->getStaticPressure();
            auto pressPitot  = sensors->getPitotLastSample();

            tm.timestamp       = TimestampTimer::getTimestamp();
            tm.pressure_digi   = pressDigi.pressure;
            tm.pressure_static = pressStatic.pressure;
            tm.airspeed_pitot  = pressPitot.airspeed;
            tm.altitude_agl    = -1.0f;  // TODO

            // Sensors
            tm.acc_x   = imu.accelerationX;
            tm.acc_y   = imu.accelerationY;
            tm.acc_z   = imu.accelerationZ;
            tm.gyro_x  = imu.angularSpeedX;
            tm.gyro_y  = imu.angularSpeedY;
            tm.gyro_z  = imu.angularSpeedZ;
            tm.mag_x   = mag.magneticFieldX;
            tm.mag_y   = mag.magneticFieldY;
            tm.mag_z   = mag.magneticFieldZ;
            tm.gps_lat = gps.latitude;
            tm.gps_lon = gps.longitude;
            tm.gps_alt = gps.height;
            tm.gps_fix = gps.fix;

            // Servos
            tm.left_servo_angle = getModule<Actuators>()->getServoAngle(
                ServosList::PARAFOIL_LEFT_SERVO);
            tm.right_servo_angle = getModule<Actuators>()->getServoAngle(
                ServosList::PARAFOIL_RIGHT_SERVO);

            // Algorithms
            tm.nas_n      = -1.0f;  // TODO
            tm.nas_e      = -1.0f;  // TODO
            tm.nas_d      = -1.0f;  // TODO
            tm.nas_vn     = -1.0f;  // TODO
            tm.nas_ve     = -1.0f;  // TODO
            tm.nas_vd     = -1.0f;  // TODO
            tm.nas_qx     = -1.0f;  // TODO
            tm.nas_qy     = -1.0f;  // TODO
            tm.nas_qz     = -1.0f;  // TODO
            tm.nas_qw     = -1.0f;  // TODO
            tm.nas_bias_x = -1.0f;  // TODO
            tm.nas_bias_y = -1.0f;  // TODO
            tm.nas_bias_z = -1.0f;  // TODO
            tm.wes_n      = -1.0;   // TODO
            tm.wes_e      = -1.0;   // TODO

            tm.battery_voltage     = sensors->getBatteryVoltage().voltage;
            tm.cam_battery_voltage = sensors->getCamBatteryVoltage().voltage;
            tm.temperature         = pressDigi.temperature;

            // State machines
            tm.fmm_state = static_cast<uint8_t>(
                getModule<FlightModeManager>()->getState());
            tm.nas_state = 255;  // TODO
            tm.wes_state = 255;  // TODO

            // Pins
            tm.pin_launch =
                pinHandler->getPinData(PinList::DETACH_PAYLOAD_PIN).lastState
                    ? 1
                    : 0;
            tm.pin_nosecone =
                pinHandler->getPinData(PinList::EXPULSION_SENSE).lastState ? 1
                                                                           : 0;
            tm.cutter_presence = 255;  // TODO

            mavlink_msg_payload_flight_tm_encode(config::Mavlink::SYSTEM_ID,
                                                 config::Mavlink::COMPONENT_ID,
                                                 &msg, &tm);
            enqueueMessage(msg);
            return true;
        }

        case MAV_STATS_ID:
        {
            mavlink_message_t msg;
            mavlink_payload_stats_tm_t tm;

            tm.liftoff_max_acc    = -1.0f;  // TODO
            tm.liftoff_max_acc_ts = 0;      // TODO
            tm.dpl_ts             = 0;      // TODO
            tm.dpl_max_acc        = -1.0f;  // TODO
            tm.max_z_speed        = -1.0f;  // TODO
            tm.max_z_speed_ts     = 0;      // TODO
            tm.max_airspeed_pitot = -1.0f;  // TODO
            tm.max_speed_altitude = -1.0f;  // TODO
            tm.apogee_ts          = 0;      // TODO
            tm.apogee_lat         = -1.0f;  // TODO
            tm.apogee_lon         = -1.0f;  // TODO
            tm.apogee_alt         = -1.0f;  // TODO
            tm.min_pressure       = -1.0f;  // TODO

            // Cpu stuff
            CpuMeterData cpuStats = CpuMeter::getCpuStats();
            CpuMeter::resetCpuStats();
            tm.cpu_load  = cpuStats.mean;
            tm.free_heap = cpuStats.freeHeap;

            // Log stuff
            LoggerStats loggerStats = Logger::getInstance().getStats();
            tm.log_good             = (loggerStats.lastWriteError == 0) ? 1 : 0;
            tm.log_number           = loggerStats.logNumber;

            auto canStatus       = getModule<CanHandler>()->getCanStatus();
            tm.main_board_state  = canStatus.mainState;
            tm.motor_board_state = canStatus.motorState;

            tm.main_can_status  = canStatus.isMainConnected() ? 1 : 0;
            tm.rig_can_status   = canStatus.isRigConnected() ? 1 : 0;
            tm.motor_can_status = canStatus.isMotorConnected() ? 1 : 0;

            tm.hil_state = 0;  // TODO: hil

            mavlink_msg_payload_stats_tm_encode(config::Mavlink::SYSTEM_ID,
                                                config::Mavlink::COMPONENT_ID,
                                                &msg, &tm);
            enqueueMessage(msg);
            return true;
        }

        default:
            return false;
    }
}

bool Radio::enqueueSensorsTm(SensorsTMList tmId)
{
    switch (tmId)
    {
        case MAV_GPS_ID:
        {
            mavlink_message_t msg;
            mavlink_gps_tm_t tm;

            auto sample = getModule<Sensors>()->getUBXGPSLastSample();

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

            mavlink_msg_gps_tm_encode(config::Mavlink::SYSTEM_ID,
                                      config::Mavlink::COMPONENT_ID, &msg, &tm);
            enqueueMessage(msg);
            return true;
        }

        case MAV_ADS131M08_ID:
        {
            mavlink_message_t msg;
            mavlink_adc_tm_t tm;

            auto sample = getModule<Sensors>()->getADS131M08LastSample();

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

            mavlink_msg_adc_tm_encode(config::Mavlink::SYSTEM_ID,
                                      config::Mavlink::COMPONENT_ID, &msg, &tm);
            enqueueMessage(msg);
            return true;
        }

        case MAV_BATTERY_VOLTAGE_ID:
        {
            mavlink_message_t msg;
            mavlink_voltage_tm_t tm;

            auto data = getModule<Sensors>()->getBatteryVoltage();

            tm.voltage   = data.voltage;
            tm.timestamp = data.voltageTimestamp;
            strcpy(tm.sensor_name, "BATTERY_VOLTAGE");

            mavlink_msg_voltage_tm_encode(config::Mavlink::SYSTEM_ID,
                                          config::Mavlink::COMPONENT_ID, &msg,
                                          &tm);
            enqueueMessage(msg);
            return true;
        }

        case MAV_LPS28DFW_ID:
        {
            mavlink_message_t msg;
            mavlink_pressure_tm_t tm;

            auto sample = getModule<Sensors>()->getLPS28DFWLastSample();

            tm.pressure  = sample.pressure;
            tm.timestamp = sample.pressureTimestamp;
            strcpy(tm.sensor_name, "LPS28DFW");

            mavlink_msg_pressure_tm_encode(config::Mavlink::SYSTEM_ID,
                                           config::Mavlink::COMPONENT_ID, &msg,
                                           &tm);

            enqueueMessage(msg);
            return true;
        }

        case MAV_LPS22DF_ID:
        {
            mavlink_message_t msg;
            mavlink_pressure_tm_t tm;

            auto sample = getModule<Sensors>()->getLPS22DFLastSample();

            tm.pressure  = sample.pressure;
            tm.timestamp = sample.pressureTimestamp;
            strcpy(tm.sensor_name, "LPS22DF");

            mavlink_msg_pressure_tm_encode(config::Mavlink::SYSTEM_ID,
                                           config::Mavlink::COMPONENT_ID, &msg,
                                           &tm);

            enqueueMessage(msg);
            return true;
        }

        case MAV_LIS2MDL_ID:
        {
            mavlink_message_t msg;
            mavlink_imu_tm_t tm;

            auto sample = getModule<Sensors>()->getLIS2MDLLastSample();

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

            mavlink_msg_imu_tm_encode(config::Mavlink::SYSTEM_ID,
                                      config::Mavlink::COMPONENT_ID, &msg, &tm);
            enqueueMessage(msg);
            return true;
        }

        case MAV_LSM6DSRX_ID:
        {
            mavlink_message_t msg;
            mavlink_imu_tm_t tm;

            auto sample = getModule<Sensors>()->getLSM6DSRXLastSample();

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

            mavlink_msg_imu_tm_encode(config::Mavlink::SYSTEM_ID,
                                      config::Mavlink::COMPONENT_ID, &msg, &tm);
            enqueueMessage(msg);
            return true;
        }

        case MAV_H3LIS331DL_ID:
        {
            mavlink_message_t msg;
            mavlink_imu_tm_t tm;

            auto sample = getModule<Sensors>()->getH3LIS331DLLastSample();

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

            mavlink_msg_imu_tm_encode(config::Mavlink::SYSTEM_ID,
                                      config::Mavlink::COMPONENT_ID, &msg, &tm);
            enqueueMessage(msg);
            return true;
        }

        default:
            return false;
    }
}

}  // namespace Payload
