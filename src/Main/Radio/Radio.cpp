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

#include <Main/PersistentVars/PersistentVars.h>
#include <common/Events.h>
#include <common/Radio.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
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
        instance->handleDioIRQ();
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
    TaskScheduler& scheduler = getModule<BoardScheduler>()->getRadioScheduler();

    // Setup the frontend
    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<Skyward433Frontend>();

    // Setup transceiver
    radio = std::make_unique<SX1278Fsk>(
        getModule<Buses>()->getRadio(), radio::cs::getPin(),
        radio::dio0::getPin(), radio::dio1::getPin(), radio::dio3::getPin(),
        SPI::ClockDivider::DIV_64, std::move(frontend));

    // Store the global radio instance
    setIRQRadio(radio.get());

    // Initialize radio
    if (radio->init(MAIN_RADIO_CONFIG) != SX1278Fsk::Error::NONE)
    {
        LOG_ERR(logger, "Failed to initialize Main radio");
        return false;
    }

    // Initialize mavdriver
    mavDriver = std::make_unique<MavDriver>(
        radio.get(), [this](MavDriver*, const mavlink_message_t& msg)
        { handleMessage(msg); }, Config::Radio::MAV_SLEEP_AFTER_SEND,
        Config::Radio::MAV_OUT_BUFFER_MAX_AGE);

    if (!mavDriver->start())
    {
        LOG_ERR(logger, "Failed to initialize Main mav driver");
        return false;
    }

    // High rate periodic telemetry
    uint8_t result = scheduler.addTask(
        [this]()
        {
            enqueueSystemTm(SystemTMList::MAV_MOTOR_ID);
            enqueueSystemTm(SystemTMList::MAV_FLIGHT_ID);
            flushPackets();
        },
        Config::Radio::HIGH_RATE_TELEMETRY);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add periodic telemetry task");
        return false;
    }

    // Low rate periodic telemetry
    result = scheduler.addTask([this]()
                               { enqueueSystemTm(SystemTMList::MAV_STATS_ID); },
                               Config::Radio::LOW_RATE_TELEMETRY);

    if (result == 0)
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
    size_t count =
        std::min(queuedPackets.count(), Config::Radio::MAX_PACKETS_PER_FLUSH);
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

void Radio::enqueueNack(const mavlink_message_t& msg, uint8_t errorId)
{
    mavlink_message_t nackMsg;
    mavlink_msg_nack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                             Config::Radio::MAV_COMPONENT_ID, &nackMsg,
                             msg.msgid, msg.seq, errorId);
    enqueuePacket(nackMsg);
}

void Radio::enqueueWack(const mavlink_message_t& msg, uint8_t errorId)
{
    mavlink_message_t wackMsg;
    mavlink_msg_wack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                             Config::Radio::MAV_COMPONENT_ID, &wackMsg,
                             msg.msgid, msg.seq, errorId);
    enqueuePacket(wackMsg);
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
                enqueueAck(msg);
            else
                enqueueNack(msg, 0);

            break;
        }

        case MAVLINK_MSG_ID_SENSOR_TM_REQUEST_TC:
        {
            uint8_t tmId =
                mavlink_msg_sensor_tm_request_tc_get_sensor_name(&msg);
            if (enqueueSensorsTm(tmId))
                enqueueAck(msg);
            else
                enqueueNack(msg, 0);

            break;
        }

        case MAVLINK_MSG_ID_WIGGLE_SERVO_TC:
        {
            ServosList servoId = static_cast<ServosList>(
                mavlink_msg_wiggle_servo_tc_get_servo_id(&msg));

            if (getModule<FlightModeManager>()->getState() ==
                    FlightModeManagerState::TEST_MODE ||
                getModule<FlightModeManager>()->getState() ==
                    FlightModeManagerState::LANDED)
            {
                // If the state is test mode, the wiggle is done
                getModule<Actuators>()->wiggleServo(servoId);
                enqueueAck(msg);
            }
            else
            {
                enqueueNack(msg, 0);
            }

            break;
        }

        case MAVLINK_MSG_ID_SET_SERVO_ANGLE_TC:
        {
            ServosList servoId = static_cast<ServosList>(
                mavlink_msg_set_servo_angle_tc_get_servo_id(&msg));

            if (getModule<FlightModeManager>()->getState() ==
                    FlightModeManagerState::TEST_MODE &&
                servoId == ServosList::AIR_BRAKES_SERVO)
            {
                float angle = mavlink_msg_set_servo_angle_tc_get_angle(&msg);

                getModule<Actuators>()->setAbkPosition(angle);
                enqueueAck(msg);
            }
            else
            {
                enqueueNack(msg, 0);
            }

            break;
        }

        case MAVLINK_MSG_ID_RESET_SERVO_TC:
        {
            ServosList servoId = static_cast<ServosList>(
                mavlink_msg_set_servo_angle_tc_get_servo_id(&msg));

            if (getModule<FlightModeManager>()->getState() ==
                    FlightModeManagerState::TEST_MODE &&
                servoId == ServosList::AIR_BRAKES_SERVO)
            {
                getModule<Actuators>()->setAbkPosition(0.0f);
                enqueueAck(msg);
            }
            else
            {
                enqueueNack(msg, 0);
            }

            break;
        }

        case MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC:
        {
            if (getModule<NASController>()->getState() ==
                NASControllerState::READY)
            {
                // Quaternions scalar first
                Eigen::Quaternion<float> quat{
                    mavlink_msg_set_orientation_quat_tc_get_quat_w(&msg),
                    mavlink_msg_set_orientation_quat_tc_get_quat_x(&msg),
                    mavlink_msg_set_orientation_quat_tc_get_quat_y(&msg),
                    mavlink_msg_set_orientation_quat_tc_get_quat_z(&msg)};

                float qNorm = quat.norm();

                getModule<NASController>()->setOrientation(quat.normalized());

                if (std::abs(qNorm - 1) > 0.001)
                    enqueueWack(msg, 0);
                else
                    enqueueAck(msg);
            }
            else
            {
                enqueueNack(msg, 0);
            }

            break;
        }

        default:
        {
            enqueueNack(msg, 0);
        }
    }
}

void Radio::handleCommand(const mavlink_message_t& msg)
{
    uint8_t cmdId = mavlink_msg_command_tc_get_command_id(&msg);
    switch (cmdId)
    {
        case MAV_CMD_START_LOGGING:
        {
            if (Logger::getInstance().start())
            {
                Logger::getInstance().resetStats();
                enqueueAck(msg);
            }
            else
            {
                enqueueNack(msg, 0);
            }
            break;
        }

        case MAV_CMD_STOP_LOGGING:
        {
            Logger::getInstance().stop();
            break;
        }

        case MAV_CMD_SAVE_CALIBRATION:
        {
            if (getModule<FlightModeManager>()->getState() ==
                FlightModeManagerState::TEST_MODE)
            {
                if (getModule<Sensors>()->saveMagCalibration())
                    enqueueAck(msg);
                else
                    enqueueNack(msg, 0);
            }
            else
            {
                enqueueNack(msg, 0);
            }

            break;
        }

        default:
        {
            // Try to map the command to an event
            auto ev = mavCmdToEvent(cmdId);
            if (ev != LAST_EVENT)
            {
                EventBroker::getInstance().post(ev, TOPIC_TMTC);
                enqueueAck(msg);
            }
            else
            {
                enqueueNack(msg, 0);
            }
        }
    }
}

bool Radio::enqueueSystemTm(uint8_t tmId)
{
    switch (tmId)
    {
        case MAV_PIN_OBS_ID:
        {
            constexpr std::array<PinHandler::PinList, 5> PIN_LIST = {
                PinHandler::PinList::RAMP_PIN,
                PinHandler::PinList::DETACH_MAIN_PIN,
                PinHandler::PinList::DETACH_PAYLOAD_PIN,
                PinHandler::PinList::EXPULSION_SENSE,
                PinHandler::PinList::CUTTER_SENSE,
            };

            PinHandler* pinHandler = getModule<PinHandler>();
            for (auto pin : PIN_LIST)
            {
                mavlink_message_t msg;
                mavlink_pin_tm_t tm;

                auto pinData = pinHandler->getPinData(pin);

                tm.timestamp             = TimestampTimer::getTimestamp();
                tm.pin_id                = static_cast<uint8_t>(pin);
                tm.last_change_timestamp = pinData.lastStateTimestamp;
                tm.changes_counter       = pinData.changesCount;
                tm.current_state         = pinData.lastState ? 1 : 0;

                mavlink_msg_pin_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                          Config::Radio::MAV_COMPONENT_ID, &msg,
                                          &tm);
                enqueuePacket(msg);
            }

            return true;
        }

        case MAV_SENSORS_STATE_ID:
        {
            auto sensors = getModule<Sensors>()->getSensorInfos();
            for (auto sensor : sensors)
            {
                mavlink_message_t msg;
                mavlink_sensor_state_tm_t tm;

                strcpy(tm.sensor_name, sensor.id.c_str());
                tm.initialized = sensor.isInitialized ? 1 : 0;
                tm.enabled     = sensor.isEnabled ? 1 : 0;

                mavlink_msg_sensor_state_tm_encode(
                    Config::Radio::MAV_SYSTEM_ID,
                    Config::Radio::MAV_COMPONENT_ID, &msg, &tm);
                enqueuePacket(msg);
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

            mavlink_msg_sys_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            enqueuePacket(msg);
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

            mavlink_msg_logger_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                         Config::Radio::MAV_COMPONENT_ID, &msg,
                                         &tm);
            enqueuePacket(msg);
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

            mavlink_msg_mavlink_stats_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                                Config::Radio::MAV_COMPONENT_ID,
                                                &msg, &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_REFERENCE_ID:
        {
            mavlink_message_t msg;
            mavlink_reference_tm_t tm;

            ReferenceValues ref =
                getModule<AlgoReference>()->getReferenceValues();

            tm.timestamp       = TimestampTimer::getTimestamp();
            tm.ref_altitude    = ref.refAltitude;
            tm.ref_pressure    = ref.refPressure;
            tm.ref_temperature = ref.refTemperature;
            tm.ref_latitude    = ref.refLatitude;
            tm.ref_longitude   = ref.refLongitude;
            tm.msl_pressure    = ref.mslPressure;
            tm.msl_temperature = ref.mslTemperature;

            mavlink_msg_reference_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                            Config::Radio::MAV_COMPONENT_ID,
                                            &msg, &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_CALIBRATION_ID:
        {
            mavlink_message_t msg;
            mavlink_calibration_tm_t tm;

            CalibrationData data = getModule<Sensors>()->getCalibration();

            tm.timestamp            = data.timestamp;
            tm.gyro_bias_x          = data.gyroBiasX;
            tm.gyro_bias_y          = data.gyroBiasY;
            tm.gyro_bias_z          = data.gyroBiasZ;
            tm.mag_bias_x           = data.magBiasX;
            tm.mag_bias_y           = data.magBiasY;
            tm.mag_bias_z           = data.magBiasZ;
            tm.mag_scale_x          = data.magScaleX;
            tm.mag_scale_y          = data.magScaleY;
            tm.mag_scale_z          = data.magScaleZ;
            tm.static_press_1_bias  = -1.0f;  // TODO: remove in mavlink
            tm.static_press_1_scale = -1.0f;  // TODO: remove in mavlink
            tm.static_press_2_bias  = -1.0f;  // TODO: remove in mavlink
            tm.static_press_2_scale = -1.0f;  // TODO: remove in mavlink
            tm.dpl_bay_press_bias   = -1.0f;  // TODO: remove in mavlink
            tm.dpl_bay_press_scale  = -1.0f;  // TODO: remove in mavlink

            mavlink_msg_calibration_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                              Config::Radio::MAV_COMPONENT_ID,
                                              &msg, &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_ADA_ID:
        {
            mavlink_message_t msg;
            mavlink_ada_tm_t tm;

            // Get the current ADA state
            ADAController* ada = getModule<ADAController>();

            ADAState state = ada->getADAState();
            ReferenceValues ref =
                getModule<AlgoReference>()->getReferenceValues();

            tm.timestamp       = state.timestamp;
            tm.state           = static_cast<uint8_t>(ada->getState());
            tm.kalman_x0       = state.x0;
            tm.kalman_x1       = state.x1;
            tm.kalman_x2       = state.x2;
            tm.vertical_speed  = state.verticalSpeed;
            tm.msl_altitude    = state.mslAltitude;
            tm.msl_pressure    = ref.mslPressure;
            tm.msl_temperature = ref.mslTemperature - 273.15f;
            tm.ref_altitude    = ref.refAltitude;
            tm.ref_temperature = ref.refTemperature - 273.15f;
            tm.ref_pressure    = ref.refPressure;
            tm.dpl_altitude    = ada->getDeploymentAltitude();

            mavlink_msg_ada_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_NAS_ID:
        {
            mavlink_message_t msg;
            mavlink_nas_tm_t tm;

            // Get the current NAS state
            NASController* nas = getModule<NASController>();

            NASState state = nas->getNASState();
            ReferenceValues ref =
                getModule<AlgoReference>()->getReferenceValues();

            tm.timestamp       = state.timestamp;
            tm.state           = static_cast<uint8_t>(nas->getState());
            tm.nas_n           = state.n;
            tm.nas_e           = state.e;
            tm.nas_d           = state.d;
            tm.nas_vn          = state.vn;
            tm.nas_ve          = state.ve;
            tm.nas_vd          = state.vd;
            tm.nas_qx          = state.qx;
            tm.nas_qy          = state.qy;
            tm.nas_qz          = state.qz;
            tm.nas_qw          = state.qw;
            tm.nas_bias_x      = state.bx;
            tm.nas_bias_y      = state.by;
            tm.nas_bias_z      = state.bz;
            tm.ref_pressure    = ref.refPressure;
            tm.ref_temperature = ref.refTemperature - 273.15f;
            tm.ref_latitude    = ref.refLatitude;
            tm.ref_longitude   = ref.refLongitude;

            mavlink_msg_nas_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_FLIGHT_ID:
        {
            mavlink_message_t msg;
            mavlink_rocket_flight_tm_t tm;

            Sensors* sensors       = getModule<Sensors>();
            Actuators* actuators   = getModule<Actuators>();
            ADAController* ada     = getModule<ADAController>();
            NASController* nas     = getModule<NASController>();
            MEAController* mea     = getModule<MEAController>();
            FlightModeManager* fmm = getModule<FlightModeManager>();

            auto imu          = sensors->getIMULastSample();
            auto gps          = sensors->getUBXGPSLastSample();
            auto vn100        = sensors->getVN100LastSample();
            auto temperature  = sensors->getTemperatureLastSample();
            auto pressStatic  = sensors->getAtmosPressureLastSample();
            auto pressDpl     = sensors->getDplBayPressureLastSample();
            auto pitotStatic  = sensors->getCanPitotStaticPressLastSample();
            auto pitotDynamic = sensors->getCanPitotDynamicPressLastSample();
            auto adaState     = ada->getADAState();
            auto nasState     = nas->getNASState();
            auto meaState     = mea->getMEAState();
            auto ref = getModule<AlgoReference>()->getReferenceValues();

            // Compute airspeed
            float airspeedPitot =
                (pitotDynamic.pressure > 0
                     ? Aeroutils::computePitotAirspeed(
                           pitotStatic.pressure + pitotDynamic.pressure,
                           pitotStatic.pressure, nasState.d, ref.refTemperature)
                     : 0);

            tm.timestamp = TimestampTimer::getTimestamp();

            tm.airspeed_pitot = airspeedPitot;
            tm.mea_mass       = meaState.estimatedMass;
            tm.mea_apogee     = meaState.estimatedApogee;

            // Sensors
            tm.pressure_digi   = -1.0f;  // TODO: rmeove in mavlink
            tm.pressure_static = pressStatic.pressure;
            tm.pressure_dpl    = pressDpl.pressure;

            tm.acc_x = imu.accelerationX;
            tm.acc_y = imu.accelerationY;
            tm.acc_z = imu.accelerationZ;

            tm.gyro_x = imu.angularSpeedX;
            tm.gyro_y = imu.angularSpeedY;
            tm.gyro_z = imu.angularSpeedZ;

            tm.mag_x = imu.magneticFieldX;
            tm.mag_y = imu.magneticFieldY;
            tm.mag_z = imu.magneticFieldZ;

            tm.gps_alt = gps.height;
            tm.gps_lat = gps.latitude;
            tm.gps_lon = gps.longitude;
            tm.gps_fix = gps.fix;

            tm.vn100_qx = vn100.quaternionX;
            tm.vn100_qy = vn100.quaternionY;
            tm.vn100_qz = vn100.quaternionZ;
            tm.vn100_qw = vn100.quaternionW;

            // Actuators
            tm.abk_angle =
                actuators->getServoPosition(ServosList::AIR_BRAKES_SERVO);

            // Algorithms
            tm.nas_n        = nasState.n;
            tm.nas_e        = nasState.e;
            tm.nas_d        = nasState.d;
            tm.nas_vn       = nasState.vn;
            tm.nas_ve       = nasState.ve;
            tm.nas_vd       = nasState.vd;
            tm.nas_qx       = nasState.qx;
            tm.nas_qy       = nasState.qy;
            tm.nas_qz       = nasState.qz;
            tm.nas_qw       = nasState.qw;
            tm.nas_bias_x   = nasState.bx;
            tm.nas_bias_y   = nasState.by;
            tm.nas_bias_z   = nasState.bz;
            tm.altitude_agl = -nasState.d;

            tm.fmm_state = static_cast<uint8_t>(fmm->getState());

            tm.pressure_ada   = adaState.x0;
            tm.ada_vert_speed = adaState.verticalSpeed;

            tm.battery_voltage = sensors->getBatteryVoltageLastSample().voltage;
            tm.cam_battery_voltage =
                sensors->getCamBatteryVoltageLastSample().voltage;
            tm.temperature = temperature.temperature;

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

            PinHandler* pinHandler  = getModule<PinHandler>();
            ADAController* ada      = getModule<ADAController>();
            NASController* nas      = getModule<NASController>();
            MEAController* mea      = getModule<MEAController>();
            ABKController* abk      = getModule<ABKController>();
            Actuators* actuators    = getModule<Actuators>();
            StatsRecorder* recorder = getModule<StatsRecorder>();

            tm.timestamp = TimestampTimer::getTimestamp();

            // General flight stats
            StatsRecorder::Stats stats = recorder->getStats();
            tm.liftoff_ts              = stats.liftoffTs;
            tm.liftoff_max_acc         = stats.liftoffMaxAcc;
            tm.liftoff_max_acc_ts      = stats.liftoffMaxAccTs;
            tm.shutdown_ts             = stats.shutdownTs;
            tm.shutdown_alt            = stats.shutdownAlt;
            tm.max_speed_ts            = stats.maxSpeedTs;
            tm.max_speed               = stats.maxSpeed;
            tm.max_speed_altitude      = stats.maxSpeedAlt;
            tm.max_mach_ts             = stats.maxMachTs;
            tm.max_mach                = stats.maxMach;
            tm.apogee_ts               = stats.apogeeTs;
            tm.apogee_lat              = stats.apogeeLat;
            tm.apogee_lon              = stats.apogeeLon;
            tm.apogee_alt              = stats.apogeeAlt;
            tm.apogee_max_acc_ts       = stats.apogeeMaxAccTs;
            tm.apogee_max_acc          = stats.apogeeMaxAcc;
            tm.dpl_ts                  = stats.dplTs;
            tm.dpl_alt                 = stats.dplAlt;
            tm.dpl_max_acc_ts          = stats.dplMaxAccTs;
            tm.dpl_max_acc             = stats.dplMaxAcc;
            tm.dpl_bay_max_pressure_ts = stats.maxDplPressureTs;
            tm.dpl_bay_max_pressure    = stats.maxDplPressure;

            // Algorithms reference
            auto ref   = getModule<AlgoReference>()->getReferenceValues();
            tm.ref_lat = ref.refLatitude;
            tm.ref_lon = ref.refLongitude;
            tm.ref_alt = ref.refAltitude;

            // Cpu stuff
            CpuMeterData cpuStats = CpuMeter::getCpuStats();
            CpuMeter::resetCpuStats();
            tm.cpu_load  = cpuStats.mean;
            tm.free_heap = cpuStats.freeHeap;

            // Also log this to the SD
            sdLogger.log(cpuStats);

            // FMM states
            tm.ada_state = static_cast<uint8_t>(ada->getState());
            tm.abk_state = static_cast<uint8_t>(abk->getState());
            tm.nas_state = static_cast<uint8_t>(nas->getState());
            tm.mea_state = static_cast<uint8_t>(mea->getState());

            // Actuators
            tm.exp_angle =
                actuators->getServoPosition(ServosList::EXPULSION_SERVO);

            tm.pin_launch =
                pinHandler->getPinData(PinHandler::PinList::RAMP_PIN).lastState
                    ? 1
                    : 0;
            tm.pin_nosecone =
                pinHandler->getPinData(PinHandler::PinList::DETACH_MAIN_PIN)
                        .lastState
                    ? 1
                    : 0;
            tm.pin_expulsion =
                pinHandler->getPinData(PinHandler::PinList::EXPULSION_SENSE)
                        .lastState
                    ? 1
                    : 0;
            tm.cutter_presence =
                pinHandler->getPinData(PinHandler::PinList::CUTTER_SENSE)
                        .lastState
                    ? 1
                    : 0;

            // Log stuff
            LoggerStats loggerStats = Logger::getInstance().getStats();
            tm.log_good             = (loggerStats.lastWriteError == 0) ? 1 : 0;
            tm.log_number           = loggerStats.logNumber;

            CanHandler::CanStatus canStatus =
                getModule<CanHandler>()->getCanStatus();
            tm.payload_board_state = canStatus.getPayloadState();
            tm.motor_board_state   = canStatus.getMotorState();

            tm.payload_can_status = canStatus.isPayloadConnected() ? 1 : 0;
            tm.motor_can_status   = canStatus.isMotorConnected() ? 1 : 0;
            tm.rig_can_status     = canStatus.isRigConnected() ? 1 : 0;

            tm.hil_state = PersistentVars::getHilMode() ? 1 : 0;

            mavlink_msg_rocket_stats_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                               Config::Radio::MAV_COMPONENT_ID,
                                               &msg, &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_MOTOR_ID:
        {
            mavlink_message_t msg;
            mavlink_motor_tm_t tm;

            Sensors* sensors     = getModule<Sensors>();
            Actuators* actuators = getModule<Actuators>();

            tm.timestamp = TimestampTimer::getTimestamp();

            // Sensors (either CAN or local)
            tm.top_tank_pressure =
                sensors->getCanTopTankPressLastSample().pressure;
            tm.bottom_tank_pressure =
                sensors->getCanBottomTankPressLastSample().pressure;
            tm.combustion_chamber_pressure =
                sensors->getCanCCPressLastSample().pressure;
            tm.tank_temperature =
                sensors->getCanTankTempLastSample().temperature;
            tm.battery_voltage =
                sensors->getCanMotorBatteryVoltageLastSample().voltage;

            // Valve states
            tm.main_valve_state =
                actuators->isCanServoOpen(ServosList::MAIN_VALVE) ? 1 : 0;
            tm.venting_valve_state =
                actuators->isCanServoOpen(ServosList::VENTING_VALVE) ? 1 : 0;

            // Can data
            CanHandler::CanStatus canStatus =
                getModule<CanHandler>()->getCanStatus();

            tm.log_number = canStatus.getMotorLogNumber();
            tm.log_good   = canStatus.isMotorLogGood() ? 1 : 0;
            tm.hil_state  = canStatus.isMotorHil() ? 1 : 0;

            mavlink_msg_motor_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                        Config::Radio::MAV_COMPONENT_ID, &msg,
                                        &tm);
            enqueuePacket(msg);
            return true;

            return true;
        }
        default:
            return false;
    }
}

bool Radio::enqueueSensorsTm(uint8_t tmId)
{
    switch (tmId)
    {
        case MAV_GPS_ID:
        {
            mavlink_message_t msg;

            auto sample = getModule<Sensors>()->getUBXGPSLastSample();

            mavlink_gps_tm_t tm;
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

        case MAV_BATTERY_VOLTAGE_ID:
        {
            mavlink_message_t msg;

            auto data = getModule<Sensors>()->getBatteryVoltageLastSample();

            mavlink_voltage_tm_t tm;
            tm.voltage   = data.voltage;
            tm.timestamp = data.voltageTimestamp;
            strcpy(tm.sensor_name, "BatteryVoltage");

            mavlink_msg_voltage_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                          Config::Radio::MAV_COMPONENT_ID, &msg,
                                          &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_LPS22DF_ID:
        {
            mavlink_message_t msg;

            auto sample = getModule<Sensors>()->getLPS22DFLastSample();

            mavlink_pressure_tm_t tm1;
            tm1.pressure  = sample.pressure;
            tm1.timestamp = sample.pressureTimestamp;
            strcpy(tm1.sensor_name, "LPS22DF");

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm1);
            enqueuePacket(msg);

            mavlink_temp_tm_t tm2;
            tm2.temperature = sample.temperature;
            tm2.timestamp   = sample.temperatureTimestamp;
            strcpy(tm2.sensor_name, "LPS22DF");

            mavlink_msg_temp_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                       Config::Radio::MAV_COMPONENT_ID, &msg,
                                       &tm2);
            enqueuePacket(msg);

            return true;
        }

        case MAV_LIS2MDL_ID:
        {
            mavlink_message_t msg;

            auto sample = getModule<Sensors>()->getLIS2MDLLastSample();

            mavlink_imu_tm_t tm1;
            tm1.acc_x     = -1.0f;
            tm1.acc_y     = -1.0f;
            tm1.acc_z     = -1.0f;
            tm1.gyro_x    = -1.0f;
            tm1.gyro_y    = -1.0f;
            tm1.gyro_z    = -1.0f;
            tm1.mag_x     = sample.magneticFieldX;
            tm1.mag_y     = sample.magneticFieldY;
            tm1.mag_z     = sample.magneticFieldZ;
            tm1.timestamp = sample.magneticFieldTimestamp;
            strcpy(tm1.sensor_name, "LIS2MDL");

            mavlink_msg_imu_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm1);
            enqueuePacket(msg);

            mavlink_temp_tm_t tm2;
            tm2.temperature = sample.temperature;
            tm2.timestamp   = sample.temperatureTimestamp;
            strcpy(tm2.sensor_name, "LIS2MDL");

            mavlink_msg_temp_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                       Config::Radio::MAV_COMPONENT_ID, &msg,
                                       &tm2);
            enqueuePacket(msg);

            return true;
        }

        case MAV_LSM6DSRX_ID:
        {
            {
                mavlink_message_t msg;

                auto sample = getModule<Sensors>()->getLSM6DSRX0LastSample();

                mavlink_imu_tm_t tm;
                tm.mag_x     = -1.0f;
                tm.mag_y     = -1.0f;
                tm.mag_z     = -1.0f;
                tm.acc_x     = sample.accelerationX;
                tm.acc_y     = sample.accelerationY;
                tm.acc_z     = sample.accelerationZ;
                tm.gyro_x    = sample.angularSpeedX;
                tm.gyro_y    = sample.angularSpeedY;
                tm.gyro_z    = sample.angularSpeedZ;
                tm.timestamp = sample.accelerationTimestamp;
                strcpy(tm.sensor_name, "LSM6DSRX_0");

                mavlink_msg_imu_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                          Config::Radio::MAV_COMPONENT_ID, &msg,
                                          &tm);
                enqueuePacket(msg);
            }

            {
                mavlink_message_t msg;

                auto sample = getModule<Sensors>()->getLSM6DSRX1LastSample();

                mavlink_imu_tm_t tm;
                tm.mag_x     = -1.0f;
                tm.mag_y     = -1.0f;
                tm.mag_z     = -1.0f;
                tm.acc_x     = sample.accelerationX;
                tm.acc_y     = sample.accelerationY;
                tm.acc_z     = sample.accelerationZ;
                tm.gyro_x    = sample.angularSpeedX;
                tm.gyro_y    = sample.angularSpeedY;
                tm.gyro_z    = sample.angularSpeedZ;
                tm.timestamp = sample.accelerationTimestamp;
                strcpy(tm.sensor_name, "LSM6DSRX_1");

                mavlink_msg_imu_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                          Config::Radio::MAV_COMPONENT_ID, &msg,
                                          &tm);
                enqueuePacket(msg);
            }

            return true;
        }

        case MAV_H3LIS331DL_ID:
        {
            mavlink_message_t msg;

            auto sample = getModule<Sensors>()->getH3LIS331DLLastSample();

            mavlink_imu_tm_t tm;
            tm.mag_x     = -1.0f;
            tm.mag_y     = -1.0f;
            tm.mag_z     = -1.0f;
            tm.gyro_x    = -1.0f;
            tm.gyro_y    = -1.0f;
            tm.gyro_z    = -1.0f;
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

        case MAV_VN100_ID:
        {
            mavlink_message_t msg;

            auto sample = getModule<Sensors>()->getVN100LastSample();

            mavlink_imu_tm_t tm1;
            tm1.mag_x     = sample.magneticFieldX;
            tm1.mag_y     = sample.magneticFieldY;
            tm1.mag_z     = sample.magneticFieldZ;
            tm1.gyro_x    = sample.angularSpeedX;
            tm1.gyro_y    = sample.angularSpeedY;
            tm1.gyro_z    = sample.angularSpeedZ;
            tm1.acc_x     = sample.accelerationX;
            tm1.acc_y     = sample.accelerationY;
            tm1.acc_z     = sample.accelerationZ;
            tm1.timestamp = sample.accelerationTimestamp;
            strcpy(tm1.sensor_name, "VN100");

            mavlink_msg_imu_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm1);
            enqueuePacket(msg);

            mavlink_attitude_tm_t tm2;
            tm2.roll      = -1.0f;
            tm2.pitch     = -1.0f;
            tm2.yaw       = -1.0f;
            tm2.quat_x    = sample.quaternionX;
            tm2.quat_y    = sample.quaternionY;
            tm2.quat_z    = sample.quaternionZ;
            tm2.quat_w    = sample.quaternionW;
            tm2.timestamp = sample.quaternionTimestamp;
            strcpy(tm2.sensor_name, "VN100");

            mavlink_msg_attitude_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm2);
            enqueuePacket(msg);

            return true;
        }

        case MAV_STATIC_PRESS_ID:
        {
            mavlink_message_t msg;

            auto sample = getModule<Sensors>()->getND015A0LastSample();

            mavlink_pressure_tm_t tm;
            tm.pressure  = sample.pressure;
            tm.timestamp = sample.pressureTimestamp;
            strcpy(tm.sensor_name, "ND015A_0");

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm);
            enqueuePacket(msg);

            return true;
        }

        case MAV_BACKUP_STATIC_PRESS_ID:
        {
            mavlink_message_t msg;

            auto sample = getModule<Sensors>()->getND015A0LastSample();

            mavlink_pressure_tm_t tm;
            tm.pressure  = sample.pressure;
            tm.timestamp = sample.pressureTimestamp;
            strcpy(tm.sensor_name, "ND015A_1");

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm);
            enqueuePacket(msg);

            return true;
        }

        case MAV_DPL_PRESS_ID:
        {
            mavlink_message_t msg;

            auto sample = getModule<Sensors>()->getDplBayPressureLastSample();

            mavlink_pressure_tm_t tm;
            tm.pressure  = sample.pressure;
            tm.timestamp = sample.pressureTimestamp;
            strcpy(tm.sensor_name, "DplBayPressure");

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm);
            enqueuePacket(msg);

            return true;
        }

        case MAV_TANK_TOP_PRESS_ID:
        {
            mavlink_message_t msg;

            auto sample = getModule<Sensors>()->getCanTopTankPressLastSample();

            mavlink_pressure_tm_t tm;
            tm.pressure  = sample.pressure;
            tm.timestamp = sample.pressureTimestamp;
            strcpy(tm.sensor_name, "TopTankPressure");

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm);
            enqueuePacket(msg);

            return true;
        }

        case MAV_TANK_BOTTOM_PRESS_ID:
        {
            mavlink_message_t msg;

            auto sample =
                getModule<Sensors>()->getCanBottomTankPressLastSample();

            mavlink_pressure_tm_t tm;
            tm.pressure  = sample.pressure;
            tm.timestamp = sample.pressureTimestamp;
            strcpy(tm.sensor_name, "BottomTankPressure");

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm);
            enqueuePacket(msg);

            return true;
        }

        case MAV_COMBUSTION_PRESS_ID:
        {
            mavlink_message_t msg;

            auto sample = getModule<Sensors>()->getCanCCPressLastSample();

            mavlink_pressure_tm_t tm;
            tm.pressure  = sample.pressure;
            tm.timestamp = sample.pressureTimestamp;
            strcpy(tm.sensor_name, "CCPressure");

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm);
            enqueuePacket(msg);

            return true;
        }

        default:
            return false;
    }
}
