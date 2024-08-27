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
#include <Payload/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Payload/PinHandler/PinHandler.h>
#include <Payload/Sensors/Sensors.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Payload/StateMachines/NASController/NASController.h>
#include <Payload/StateMachines/WingController/WingController.h>
#include <Payload/WindEstimationScheme/WindEstimation.h>
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

void Radio::MavlinkBackend::handleMessage(const mavlink_message_t& msg)
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
            bool testMode = parent.getModule<FlightModeManager>()->isTestMode();
            // Perform the wiggle in test mode only
            if (!testMode)
            {
                return enqueueNack(msg);
            }

            auto servo = static_cast<ServosList>(
                mavlink_msg_wiggle_servo_tc_get_servo_id(&msg));

            parent.getModule<Actuators>()->wiggleServo(servo);
            return enqueueAck(msg);
        }

        case MAVLINK_MSG_ID_SET_DEPLOYMENT_ALTITUDE_TC:
        {
            float altitude =
                mavlink_msg_set_deployment_altitude_tc_get_dpl_altitude(&msg);

            parent.getModule<AltitudeTrigger>()->setDeploymentAltitude(
                altitude);

            if (altitude < 0 || altitude > 3000)
            {
                return enqueueWack(msg);
            }
            else
            {
                return enqueueAck(msg);
            }
        }

        case MAVLINK_MSG_ID_RAW_EVENT_TC:
        {
            uint8_t topicId = mavlink_msg_raw_event_tc_get_topic_id(&msg);
            uint8_t eventId = mavlink_msg_raw_event_tc_get_event_id(&msg);

            bool testMode = parent.getModule<FlightModeManager>()->isTestMode();
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

void Radio::MavlinkBackend::handleCommand(const mavlink_message_t& msg)
{
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
            bool testMode = parent.getModule<FlightModeManager>()->isTestMode();
            // Save calibration data in test mode only
            if (!testMode)
            {
                return enqueueNack(msg);
            }

            bool magResult = parent.getModule<Sensors>()->saveMagCalibration();
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
            auto event = mavCmdToEvent(command);
            if (event == LAST_EVENT)
            {
                return enqueueNack(msg);
            }

            EventBroker::getInstance().post(event, TOPIC_TMTC);
            return enqueueAck(msg);
        }
    }
}

void Radio::MavlinkBackend::enqueueAck(const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(config::Mavlink::SYSTEM_ID,
                            config::Mavlink::COMPONENT_ID, &ackMsg, msg.msgid,
                            msg.seq);
    enqueueMessage(ackMsg);
}

void Radio::MavlinkBackend::enqueueNack(const mavlink_message_t& msg)
{
    mavlink_message_t nackMsg;
    mavlink_msg_nack_tm_pack(config::Mavlink::SYSTEM_ID,
                             config::Mavlink::COMPONENT_ID, &nackMsg, msg.msgid,
                             msg.seq, 0);
    enqueueMessage(nackMsg);
}

void Radio::MavlinkBackend::enqueueWack(const mavlink_message_t& msg)
{
    mavlink_message_t wackMsg;
    mavlink_msg_wack_tm_pack(config::Mavlink::SYSTEM_ID,
                             config::Mavlink::COMPONENT_ID, &wackMsg, msg.msgid,
                             msg.seq, 0);
    enqueueMessage(wackMsg);
}

bool Radio::MavlinkBackend::enqueueSystemTm(SystemTMList tmId)
{
    switch (tmId)
    {
        case MAV_SENSORS_STATE_ID:
        {
            auto sensors = parent.getModule<Sensors>()->getSensorInfo();
            for (auto sensor : sensors)
            {
                mavlink_message_t msg;
                mavlink_sensor_state_tm_t tm;

                strcpy(tm.sensor_name, sensor.id.c_str());
                tm.initialized = sensor.isInitialized;
                tm.enabled     = sensor.isEnabled;

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
            tm.logger       = Logger::getInstance().isStarted();
            tm.event_broker = EventBroker::getInstance().isRunning();
            tm.radio        = parent.isStarted();
            tm.sensors      = parent.getModule<Sensors>()->isStarted();
            tm.actuators    = parent.getModule<Actuators>()->isStarted();
            tm.pin_handler  = parent.getModule<PinHandler>()->isStarted();
            tm.can_handler  = parent.getModule<CanHandler>()->isStarted();
            tm.scheduler    = parent.getModule<BoardScheduler>()->isStarted();

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
            auto stats = Logger::getInstance().getStats();

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
            auto stats = driver->getStatus();

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

            auto* sensors = parent.getModule<Sensors>();
            auto* nas     = parent.getModule<NASController>();
            auto* wes     = parent.getModule<WindEstimation>();

            auto imu         = sensors->getLSM6DSRXLastSample();
            auto mag         = sensors->getLIS2MDLLastSample();
            auto gps         = sensors->getUBXGPSLastSample();
            auto pressDigi   = sensors->getLPS28DFWLastSample();
            auto pressStatic = sensors->getStaticPressureLastSample();
            auto nasState    = nas->getNasState();
            auto wind        = wes->getWindEstimationScheme();

            tm.timestamp       = TimestampTimer::getTimestamp();
            tm.pressure_digi   = pressDigi.pressure;
            tm.pressure_static = pressStatic.pressure;
            tm.airspeed_pitot  = -1.0f;  // TODO
            tm.altitude_agl    = -nasState.d;

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
            tm.left_servo_angle = parent.getModule<Actuators>()->getServoAngle(
                ServosList::PARAFOIL_LEFT_SERVO);
            tm.right_servo_angle = parent.getModule<Actuators>()->getServoAngle(
                ServosList::PARAFOIL_RIGHT_SERVO);

            // Algorithms
            tm.nas_n      = nasState.n;
            tm.nas_e      = nasState.e;
            tm.nas_d      = nasState.d;
            tm.nas_vn     = nasState.vn;
            tm.nas_ve     = nasState.ve;
            tm.nas_vd     = nasState.vd;
            tm.nas_qx     = nasState.qx;
            tm.nas_qy     = nasState.qy;
            tm.nas_qz     = nasState.qz;
            tm.nas_qw     = nasState.qw;
            tm.nas_bias_x = nasState.bx;
            tm.nas_bias_y = nasState.by;
            tm.nas_bias_z = nasState.bz;
            tm.wes_n      = wind[0];
            tm.wes_e      = wind[1];

            tm.battery_voltage     = sensors->getBatteryVoltage().batVoltage;
            tm.cam_battery_voltage = sensors->getCamBatteryVoltage().batVoltage;
            tm.temperature         = pressDigi.temperature;

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

            auto* fmm        = parent.getModule<FlightModeManager>();
            auto* nas        = parent.getModule<NASController>();
            auto* pinHandler = parent.getModule<PinHandler>();

            auto stats = parent.getModule<FlightStatsRecorder>()->getStats();
            auto ref   = nas->getReferenceValues();

            // Liftoff stats
            tm.liftoff_ts         = stats.liftoffTs;
            tm.liftoff_max_acc_ts = stats.liftoffMaxAccTs;
            tm.liftoff_max_acc    = stats.liftoffMaxAcc;

            // Max speed stats
            tm.max_speed_ts       = stats.maxSpeedTs;
            tm.max_mach_ts        = stats.maxMachTs;
            tm.max_speed          = stats.maxSpeed;
            tm.max_speed_altitude = stats.maxSpeedAlt;
            tm.max_mach           = stats.maxMach;

            // Apogee stats
            tm.apogee_ts         = stats.apogeeTs;
            tm.apogee_max_acc_ts = stats.apogeeMaxAccTs;
            tm.apogee_lat        = stats.apogeeLat;
            tm.apogee_lon        = stats.apogeeLon;
            tm.apogee_alt        = stats.apogeeAlt;
            tm.apogee_max_acc    = stats.apogeeMaxAcc;

            // Wing stats
            tm.wing_emc_n = -1.0f;  // TODO
            tm.wing_emc_e = -1.0f;  // TODO
            tm.wing_m1_n  = -1.0f;  // TODO
            tm.wing_m1_e  = -1.0f;  // TODO
            tm.wing_m2_n  = -1.0f;  // TODO
            tm.wing_m2_e  = -1.0f;  // TODO

            // Deployment stats
            tm.dpl_ts         = stats.dplTs;
            tm.dpl_max_acc_ts = stats.dplMaxAccTs;
            tm.dpl_alt        = stats.dplAlt;
            tm.dpl_max_acc    = stats.dplMaxAcc;

            // NAS reference values
            tm.ref_lat = ref.refLatitude;
            tm.ref_lon = ref.refLongitude;
            tm.ref_alt = ref.refAltitude;

            tm.min_pressure = stats.minPressure;

            // CPU stats
            auto cpuStats = CpuMeter::getCpuStats();
            tm.cpu_load   = cpuStats.mean;
            tm.free_heap  = cpuStats.freeHeap;

            // Logger stats
            auto loggerStats = Logger::getInstance().getStats();
            tm.log_good      = (loggerStats.lastWriteError == 0);
            tm.log_number    = loggerStats.logNumber;

            // State machines
            tm.fmm_state = static_cast<uint8_t>(fmm->getState());
            tm.nas_state = static_cast<uint8_t>(nas->getState());
            tm.wes_state = 255;  // TODO

            // Pins
            tm.pin_launch =
                pinHandler->getPinData(PinList::RAMP_DETACH_PIN).lastState;
            tm.pin_nosecone =
                pinHandler->getPinData(PinList::NOSECONE_DETACH_PIN).lastState;
            tm.cutter_presence = 255;  // TODO

            auto canStatus = parent.getModule<CanHandler>()->getCanStatus();
            tm.main_board_state  = canStatus.mainState;
            tm.motor_board_state = canStatus.motorState;

            tm.main_can_status  = canStatus.isMainConnected();
            tm.motor_can_status = canStatus.isMotorConnected();
            tm.rig_can_status   = canStatus.isRigConnected();

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

bool Radio::MavlinkBackend::enqueueSensorsTm(SensorsTMList tmId)
{
    switch (tmId)
    {
        case MAV_GPS_ID:
        {
            mavlink_message_t msg;
            mavlink_gps_tm_t tm;

            auto sample = parent.getModule<Sensors>()->getUBXGPSLastSample();

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

            auto sample = parent.getModule<Sensors>()->getADS131M08LastSample();

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

            auto data = parent.getModule<Sensors>()->getBatteryVoltage();

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

            auto sample = parent.getModule<Sensors>()->getLPS28DFWLastSample();

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

            auto sample = parent.getModule<Sensors>()->getLPS22DFLastSample();

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

            auto sample = parent.getModule<Sensors>()->getLIS2MDLLastSample();

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

            auto sample = parent.getModule<Sensors>()->getLSM6DSRXLastSample();

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

            auto sample =
                parent.getModule<Sensors>()->getH3LIS331DLLastSample();

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