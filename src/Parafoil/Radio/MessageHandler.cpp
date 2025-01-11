/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Davide Basso
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

#include <Parafoil/Actuators/Actuators.h>
#include <Parafoil/AltitudeTrigger/AltitudeTrigger.h>
#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Configs/RadioConfig.h>
#include <Parafoil/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Parafoil/PinHandler/PinHandler.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Parafoil/StateMachines/NASController/NASController.h>
#include <Parafoil/StateMachines/WingController/WingController.h>
#include <Parafoil/WindEstimation/WindEstimation.h>
#include <common/Events.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <events/EventBroker.h>

#include "Radio.h"

using namespace Boardcore;
using namespace Boardcore::Units::Angle;
using namespace Boardcore::Units::Speed;
using namespace Boardcore::Units::Length;
using namespace Boardcore::Units::Acceleration;
using namespace Common;
namespace config = Parafoil::Config::Radio;

namespace Parafoil
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
                return enqueueNack(msg);

            return enqueueAck(msg);
        }

        case MAVLINK_MSG_ID_SENSOR_TM_REQUEST_TC:
        {
            auto sensorId = static_cast<SensorsTMList>(
                mavlink_msg_sensor_tm_request_tc_get_sensor_name(&msg));

            if (!enqueueSensorsTm(sensorId))
                return enqueueNack(msg);

            return enqueueAck(msg);
        }

        case MAVLINK_MSG_ID_SERVO_TM_REQUEST_TC:
        {
            auto servo = static_cast<ServosList>(
                mavlink_msg_servo_tm_request_tc_get_servo_id(&msg));

            float position =
                parent.getModule<Actuators>()->getServoPosition(servo);
            if (position < 0)
                return enqueueNack(msg);

            mavlink_message_t tmMsg;
            mavlink_servo_tm_t tm;

            tm.servo_id       = static_cast<uint8_t>(servo);
            tm.servo_position = position;

            mavlink_msg_servo_tm_encode(config::Mavlink::SYSTEM_ID,
                                        config::Mavlink::COMPONENT_ID, &tmMsg,
                                        &tm);
            enqueueMessage(tmMsg);
            return enqueueAck(msg);
        }

        case MAVLINK_MSG_ID_SET_SERVO_ANGLE_TC:
        {
            bool allowed = parent.getModule<FlightModeManager>()->isTestMode();
            // Allow arbitrary servo movements in allowed states only
            if (!allowed)
                return enqueueNack(msg);

            auto servo = static_cast<ServosList>(
                mavlink_msg_set_servo_angle_tc_get_servo_id(&msg));
            auto position = mavlink_msg_set_servo_angle_tc_get_angle(&msg);

            if (parent.getModule<Actuators>()->setServoPosition(servo,
                                                                position))
                return enqueueAck(msg);
            else
                return enqueueNack(msg);
        }

        case MAVLINK_MSG_ID_RESET_SERVO_TC:
        {
            bool allowed = parent.getModule<FlightModeManager>()->isTestMode();
            // Reset servos in allowed states only
            if (!allowed)
                return enqueueNack(msg);

            auto servo = static_cast<ServosList>(
                mavlink_msg_reset_servo_tc_get_servo_id(&msg));

            bool reset =
                parent.getModule<Actuators>()->setServoPosition(servo, 0.0f);
            if (reset)
            {
                // One of our servos was reset
                return enqueueAck(msg);
            }

            return enqueueWack(msg);
        }

        case MAVLINK_MSG_ID_WIGGLE_SERVO_TC:
        {
            bool allowed = parent.getModule<FlightModeManager>()->isTestMode();
            // Perform the wiggle in allowed states only
            if (!allowed)
                return enqueueNack(msg);

            auto servo = static_cast<ServosList>(
                mavlink_msg_wiggle_servo_tc_get_servo_id(&msg));

            bool wiggled = parent.getModule<Actuators>()->wiggleServo(servo);
            if (wiggled)
            {
                // One of our servos was wiggled
                return enqueueAck(msg);
            }

            return enqueueWack(msg);
        }

        case MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_TC:
        {
            bool allowed = parent.getModule<FlightModeManager>()->isTestMode();
            if (!allowed)
                return enqueueNack(msg);

            float altitude =
                mavlink_msg_set_reference_altitude_tc_get_ref_altitude(&msg);

            parent.getModule<NASController>()->setReferenceAltitude(
                Meter{altitude});
            break;
        }

        case MAVLINK_MSG_ID_SET_REFERENCE_TEMPERATURE_TC:
        {
            auto allowed = parent.getModule<FlightModeManager>()->isTestMode();
            if (!allowed)
                return enqueueNack(msg);

            float temperature =
                mavlink_msg_set_reference_temperature_tc_get_ref_temp(&msg);

            parent.getModule<NASController>()->setReferenceTemperature(
                temperature);
            break;
        }

        case MAVLINK_MSG_ID_SET_COORDINATES_TC:
        {
            auto allowed = parent.getModule<FlightModeManager>()->isTestMode();
            if (!allowed)
                return enqueueNack(msg);

            float latitude = mavlink_msg_set_coordinates_tc_get_latitude(&msg);
            float longitude =
                mavlink_msg_set_coordinates_tc_get_longitude(&msg);

            parent.getModule<NASController>()->setReferenceCoordinates(
                latitude, longitude);
            break;
        }

        case MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC:
        {
            if (parent.getModule<NASController>()->getState() !=
                NASControllerState::READY)
            {
                return enqueueNack(msg);
            }

            // Scalar first quaternion, W is the first element
            auto quat = Eigen::Quaternionf{
                mavlink_msg_set_orientation_quat_tc_get_quat_w(&msg),
                mavlink_msg_set_orientation_quat_tc_get_quat_x(&msg),
                mavlink_msg_set_orientation_quat_tc_get_quat_y(&msg),
                mavlink_msg_set_orientation_quat_tc_get_quat_z(&msg),
            };

            parent.getModule<NASController>()->setOrientation(
                quat.normalized());

            float qNorm = quat.norm();
            if (std::abs(qNorm - 1) > 0.001)
                return enqueueWack(msg);
            else
                return enqueueAck(msg);
        }

        case MAVLINK_MSG_ID_SET_DEPLOYMENT_ALTITUDE_TC:
        {
            float altitude =
                mavlink_msg_set_deployment_altitude_tc_get_dpl_altitude(&msg);

            parent.getModule<AltitudeTrigger>()->setDeploymentAltitude(
                Meter{altitude});

            if (altitude < 200 || altitude > 450)
                return enqueueWack(msg);
            else
                return enqueueAck(msg);
        }

        case MAVLINK_MSG_ID_SET_TARGET_COORDINATES_TC:
        {
            float latitude =
                mavlink_msg_set_target_coordinates_tc_get_latitude(&msg);
            float longitude =
                mavlink_msg_set_target_coordinates_tc_get_longitude(&msg);

            bool targetSet =
                parent.getModule<WingController>()->setTargetCoordinates(
                    latitude, longitude);

            if (targetSet)
                return enqueueAck(msg);
            else
                return enqueueNack(msg);
        }

        case MAVLINK_MSG_ID_SET_ALGORITHM_TC:
        {
            uint8_t index =
                mavlink_msg_set_algorithm_tc_get_algorithm_number(&msg);

            bool algorithmSet =
                parent.getModule<WingController>()->selectAlgorithm(
                    static_cast<Config::Wing::AlgorithmId>(index));

            if (algorithmSet)
                return enqueueAck(msg);
            else
                return enqueueNack(msg);
        }

        case MAVLINK_MSG_ID_RAW_EVENT_TC:
        {
            uint8_t topicId = mavlink_msg_raw_event_tc_get_topic_id(&msg);
            uint8_t eventId = mavlink_msg_raw_event_tc_get_event_id(&msg);

            bool testMode = parent.getModule<FlightModeManager>()->isTestMode();
            // Raw events are allowed in test mode only
            if (!testMode)
                return enqueueNack(msg);

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
                return enqueueNack(msg);

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
                return enqueueNack(msg);

            bool magResult = parent.getModule<Sensors>()->saveMagCalibration();
            if (magResult)
                return enqueueAck(msg);
            else
                return enqueueNack(msg);
        }

        default:
        {
            // Map the command to an event and post it, if it exists
            auto event = mavCmdToEvent(command);
            if (event == LAST_EVENT)
                return enqueueNack(msg);

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
            tm.scheduler    = parent.getModule<BoardScheduler>()->isStarted();
            tm.can_handler  = false;  // Not present on parafoil board

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

        case MAV_NAS_ID:
        {
            mavlink_message_t msg;
            mavlink_nas_tm_t tm;

            auto state    = parent.getModule<NASController>()->getState();
            auto nasState = parent.getModule<NASController>()->getNasState();
            auto ref = parent.getModule<NASController>()->getReferenceValues();

            tm.timestamp       = nasState.timestamp;
            tm.nas_n           = nasState.n;
            tm.nas_e           = nasState.e;
            tm.nas_d           = nasState.d;
            tm.nas_vn          = nasState.vn;
            tm.nas_ve          = nasState.ve;
            tm.nas_vd          = nasState.vd;
            tm.nas_qx          = nasState.qx;
            tm.nas_qy          = nasState.qy;
            tm.nas_qz          = nasState.qz;
            tm.nas_qw          = nasState.qw;
            tm.nas_bias_x      = nasState.bx;
            tm.nas_bias_y      = nasState.by;
            tm.nas_bias_z      = nasState.bz;
            tm.ref_pressure    = ref.refPressure;
            tm.ref_temperature = ref.refTemperature;
            tm.ref_latitude    = ref.refLatitude;
            tm.ref_longitude   = ref.refLongitude;
            tm.state           = static_cast<uint8_t>(state);

            mavlink_msg_nas_tm_encode(config::Mavlink::SYSTEM_ID,
                                      config::Mavlink::COMPONENT_ID, &msg, &tm);
            enqueueMessage(msg);
            return true;
        }

        case MAV_FLIGHT_ID:
        {
            mavlink_message_t msg;
            mavlink_payload_flight_tm_t tm;

            auto* sensors = parent.getModule<Sensors>();
            auto* nas     = parent.getModule<NASController>();
            auto* fmm     = parent.getModule<FlightModeManager>();

            auto imu      = sensors->getBMX160WithCorrectionLastSample();
            auto gps      = sensors->getUBXGPSLastSample();
            auto lps22df  = sensors->getLPS22DFLastSample();
            auto nasState = nas->getNasState();
            // auto ref      = nas->getReferenceValues();

            tm.timestamp = TimestampTimer::getTimestamp();
            // No digital pressure sensor, use static
            tm.pressure_digi   = lps22df.pressure;
            tm.pressure_static = lps22df.pressure;
            // No dynamic pressure sensor
            tm.pressure_dynamic = -1.f;
            // Not present on parafoil board
            tm.airspeed_pitot = 0;
            tm.altitude_agl   = -nasState.d;

            // Sensors
            tm.acc_x   = imu.accelerationX;
            tm.acc_y   = imu.accelerationY;
            tm.acc_z   = imu.accelerationZ;
            tm.gyro_x  = imu.angularSpeedX;
            tm.gyro_y  = imu.angularSpeedY;
            tm.gyro_z  = imu.angularSpeedZ;
            tm.mag_x   = imu.magneticFieldX;
            tm.mag_y   = imu.magneticFieldY;
            tm.mag_z   = imu.magneticFieldZ;
            tm.gps_lat = gps.latitude;
            tm.gps_lon = gps.longitude;
            tm.gps_alt = gps.height;
            tm.gps_fix = gps.fix;

            // Servos
            tm.left_servo_angle =
                parent.getModule<Actuators>()
                    ->getServoAngle(ServosList::PARAFOIL_LEFT_SERVO)
                    .value<Degree>();
            tm.right_servo_angle =
                parent.getModule<Actuators>()
                    ->getServoAngle(ServosList::PARAFOIL_RIGHT_SERVO)
                    .value<Degree>();

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

            // Wind estimation
            auto wind = parent.getModule<WindEstimation>()->getPrediction();

            tm.wes_n = wind.vn.value<MeterPerSecond>();
            tm.wes_e = wind.ve.value<MeterPerSecond>();

            tm.battery_voltage = sensors->getBatteryVoltage().batVoltage;
            // No integrated camera
            tm.cam_battery_voltage = -1.f;
            tm.temperature         = lps22df.temperature;

            // State machines
            tm.fmm_state = static_cast<uint8_t>(fmm->getState());

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

            auto* nas        = parent.getModule<NASController>();
            auto* wnc        = parent.getModule<WingController>();
            auto* pinHandler = parent.getModule<PinHandler>();
            auto* recorder   = parent.getModule<FlightStatsRecorder>();
            auto& logger     = Logger::getInstance();

            auto stats            = recorder->getStats();
            auto ref              = nas->getReferenceValues();
            auto wingTarget       = wnc->getTargetCoordinates();
            auto wingActiveTarget = wnc->getActiveTarget();
            auto wingAlgorithm    = wnc->getSelectedAlgorithm();
            auto cpuStats         = CpuMeter::getCpuStats();
            auto loggerStats      = logger.getStats();

            // Log CPU stats and reset them
            CpuMeter::resetCpuStats();
            logger.log(cpuStats);

            tm.timestamp = TimestampTimer::getTimestamp();

            // No liftoff stats for parafoil, used for drop time
            tm.liftoff_ts         = stats.dropTs;
            tm.liftoff_max_acc_ts = stats.dropMaxAccTs;
            tm.liftoff_max_acc =
                stats.dropMaxAcc.value<MeterPerSecondSquared>();

            // Max speed stats
            tm.max_speed_ts       = stats.maxSpeedTs;
            tm.max_speed          = stats.maxSpeed.value<MeterPerSecond>();
            tm.max_speed_altitude = stats.maxSpeedAlt.value<Meter>();

            // Useless for parafoil
            tm.max_mach_ts = 0;
            tm.max_mach    = -1.f;

            // No apogee stats for parafoil
            tm.apogee_ts         = -1;
            tm.apogee_max_acc_ts = 0;
            tm.apogee_lat        = -1.f;
            tm.apogee_lon        = -1.f;
            tm.apogee_alt        = -1.f;
            tm.apogee_max_acc    = -1.f;

            // Wing stats
            tm.wing_target_lat      = wingTarget.x();
            tm.wing_target_lon      = wingTarget.y();
            tm.wing_active_target_n = wingActiveTarget.x();
            tm.wing_active_target_e = wingActiveTarget.y();
            tm.wing_algorithm       = wingAlgorithm;

            // Deployment stats
            tm.dpl_ts         = stats.dplTs;
            tm.dpl_max_acc_ts = stats.dplMaxAccTs;
            tm.dpl_alt        = stats.dplAlt.value<Meter>();
            tm.dpl_max_acc    = stats.dplMaxAcc.value<MeterPerSecondSquared>();

            // NAS reference values
            tm.ref_lat = ref.refLatitude;
            tm.ref_lon = ref.refLongitude;
            tm.ref_alt = ref.refAltitude;

            tm.min_pressure = -1.f;

            // CPU stats
            tm.cpu_load  = cpuStats.mean;
            tm.free_heap = cpuStats.freeHeap;

            // Logger stats
            tm.log_good   = (loggerStats.lastWriteError == 0);
            tm.log_number = loggerStats.logNumber;

            tm.nas_state = static_cast<uint8_t>(nas->getState());
            tm.wnc_state = static_cast<uint8_t>(wnc->getState());

            // Pins, some not present on parafoil board
            tm.pin_nosecone =
                pinHandler->getPinData(PinList::NOSECONE_PIN).lastState;
            tm.pin_launch      = 0;
            tm.cutter_presence = 0;
            // Can handler, not present on parafoil board
            tm.main_board_state  = 0;
            tm.motor_board_state = 0;

            tm.main_can_status  = 0;
            tm.motor_can_status = 0;
            tm.rig_can_status   = 0;

            tm.hil_state = 0;

            mavlink_msg_payload_stats_tm_encode(config::Mavlink::SYSTEM_ID,
                                                config::Mavlink::COMPONENT_ID,
                                                &msg, &tm);
            enqueueMessage(msg);
            return true;
        }

        case MAV_PIN_OBS_ID:
        {
            auto pinHandler = parent.getModule<PinHandler>();

            for (auto pin : PinHandler::PIN_LIST)
            {
                mavlink_message_t msg;
                mavlink_pin_tm_t tm;

                auto pinData = pinHandler->getPinData(pin);

                tm.timestamp             = TimestampTimer::getTimestamp();
                tm.pin_id                = static_cast<uint8_t>(pin);
                tm.last_change_timestamp = pinData.lastStateTimestamp;
                tm.changes_counter       = pinData.changesCount;
                tm.current_state         = pinData.lastState;

                mavlink_msg_pin_tm_encode(config::Mavlink::SYSTEM_ID,
                                          config::Mavlink::COMPONENT_ID, &msg,
                                          &tm);
                enqueueMessage(msg);
            }

            return true;
        }

        case MAV_SENSORS_STATE_ID:
        {
            auto sensors = parent.getModule<Sensors>()->getSensorInfo();
            for (auto sensor : sensors)
            {
                mavlink_message_t msg;
                mavlink_sensor_state_tm_t tm;

                constexpr size_t maxNameLen =
                    sizeof(tm.sensor_name) / sizeof(tm.sensor_name[0]) - 1;

                std::strncpy(tm.sensor_name, sensor.id.c_str(), maxNameLen);
                tm.sensor_name[maxNameLen] = '\0';  // Ensure null-termination
                tm.initialized             = sensor.isInitialized;
                tm.enabled                 = sensor.isEnabled;

                mavlink_msg_sensor_state_tm_encode(
                    config::Mavlink::SYSTEM_ID, config::Mavlink::COMPONENT_ID,
                    &msg, &tm);
                enqueueMessage(msg);
            }

            return true;
        }

        case MAV_REFERENCE_ID:
        {
            mavlink_message_t msg;
            mavlink_reference_tm_t tm;

            auto ref = parent.getModule<NASController>()->getReferenceValues();

            tm.timestamp       = TimestampTimer::getTimestamp();
            tm.ref_altitude    = ref.refAltitude;
            tm.ref_pressure    = ref.refPressure;
            tm.ref_temperature = ref.refTemperature;
            tm.ref_latitude    = ref.refLatitude;
            tm.ref_longitude   = ref.refLongitude;
            tm.msl_pressure    = ref.mslPressure;
            tm.msl_temperature = ref.mslTemperature;

            mavlink_msg_reference_tm_encode(config::Mavlink::SYSTEM_ID,
                                            config::Mavlink::COMPONENT_ID, &msg,
                                            &tm);
            enqueueMessage(msg);
            return true;
        }

        case MAV_CALIBRATION_ID:
        {
            mavlink_message_t msg;
            mavlink_calibration_tm_t tm;

            auto cal = parent.getModule<Sensors>()->getCalibrationData();

            tm.timestamp            = TimestampTimer::getTimestamp();
            tm.gyro_bias_x          = 0.0f;
            tm.gyro_bias_y          = 0.0f;
            tm.gyro_bias_z          = 0.0f;
            tm.mag_bias_x           = cal.magBiasX;
            tm.mag_bias_y           = cal.magBiasY;
            tm.mag_bias_z           = cal.magBiasZ;
            tm.mag_scale_x          = cal.magScaleX;
            tm.mag_scale_y          = cal.magScaleY;
            tm.mag_scale_z          = cal.magScaleZ;
            tm.static_press_1_bias  = 0.0f;
            tm.static_press_1_scale = 0.0f;
            tm.static_press_2_bias  = 0.0f;
            tm.static_press_2_scale = 0.0f;
            tm.dpl_bay_press_bias   = 0.0f;
            tm.dpl_bay_press_scale  = 0.0f;
            tm.dynamic_press_bias   = 0.0f;
            tm.dynamic_press_scale  = 0.0f;

            mavlink_msg_calibration_tm_encode(config::Mavlink::SYSTEM_ID,
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

        case SensorsTMList::MAV_BMX160_ID:
        {
            mavlink_message_t msg;
            mavlink_imu_tm_t tm;

            auto imuData = parent.getModule<Sensors>()
                               ->getBMX160WithCorrectionLastSample();

            strcpy(tm.sensor_name, "BMX160");
            tm.acc_x     = imuData.accelerationX;
            tm.acc_y     = imuData.accelerationY;
            tm.acc_z     = imuData.accelerationZ;
            tm.gyro_x    = imuData.angularSpeedX;
            tm.gyro_y    = imuData.angularSpeedY;
            tm.gyro_z    = imuData.angularSpeedZ;
            tm.mag_x     = imuData.magneticFieldX;
            tm.mag_y     = imuData.magneticFieldY;
            tm.mag_z     = imuData.magneticFieldZ;
            tm.timestamp = imuData.accelerationTimestamp;

            mavlink_msg_imu_tm_encode(config::Mavlink::SYSTEM_ID,
                                      config::Mavlink::COMPONENT_ID, &msg, &tm);

            return true;
        }

        case SensorsTMList::MAV_LPS22DF_ID:
        {
            mavlink_message_t msg;
            mavlink_pressure_tm_t tm;

            auto pressureData =
                parent.getModule<Sensors>()->getLPS22DFLastSample();

            tm.timestamp = pressureData.pressureTimestamp;
            strcpy(tm.sensor_name, "LPS22");
            tm.pressure = pressureData.pressure;

            mavlink_msg_pressure_tm_encode(config::Mavlink::SYSTEM_ID,
                                           config::Mavlink::COMPONENT_ID, &msg,
                                           &tm);

            return true;
        }

        case MAV_BATTERY_VOLTAGE_ID:
        {
            mavlink_message_t msg;
            mavlink_voltage_tm_t tm;

            auto data = parent.getModule<Sensors>()->getBatteryVoltage();

            tm.voltage   = data.voltage;
            tm.timestamp = data.voltageTimestamp;
            strcpy(tm.sensor_name, "BatteryVoltage");

            mavlink_msg_voltage_tm_encode(config::Mavlink::SYSTEM_ID,
                                          config::Mavlink::COMPONENT_ID, &msg,
                                          &tm);
            enqueueMessage(msg);
            return true;
        }

        case MAV_ADS131M08_ID:
        {
            mavlink_message_t msg;
            mavlink_adc_tm_t tm;

            auto sample = parent.getModule<Sensors>()->getADS131LastSample();

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

        case MAV_LIS3MDL_ID:
        {
            mavlink_message_t msg;
            mavlink_imu_tm_t tm;

            auto sample = parent.getModule<Sensors>()->getLIS3MDLLastSample();

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
            strcpy(tm.sensor_name, "LIS3MDL");

            mavlink_msg_imu_tm_encode(config::Mavlink::SYSTEM_ID,
                                      config::Mavlink::COMPONENT_ID, &msg, &tm);
            enqueueMessage(msg);
            return true;
        }

        case MAV_H3LIS331DL_ID:
        {
            mavlink_message_t msg;
            mavlink_imu_tm_t tm;

            auto sample = parent.getModule<Sensors>()->getH3LISLastSample();

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

}  // namespace Parafoil
