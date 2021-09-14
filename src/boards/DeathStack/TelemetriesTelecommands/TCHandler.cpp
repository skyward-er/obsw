/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include "TCHandler.h"

#include <AirBrakes/AirBrakesController.h>
#include <ApogeeDetectionAlgorithm/ADAController.h>
#include <DeathStack.h>
#include <configs/TMTCConfig.h>
#include <events/Events.h>

using std::map;
namespace DeathStackBoard
{

const std::map<uint8_t, uint8_t> tcMap = {
    {MAV_CMD_ARM, EV_TC_ARM},
    {MAV_CMD_DISARM, EV_TC_DISARM},

    {MAV_CMD_FORCE_INIT, EV_TC_FORCE_INIT},
    {MAV_CMD_FORCE_LAUNCH, EV_TC_LAUNCH},

    {MAV_CMD_NOSECONE_OPEN, EV_TC_NC_OPEN},
    {MAV_CMD_DPL_RESET_SERVO, EV_TC_DPL_RESET_SERVO},
    {MAV_CMD_DPL_WIGGLE_SERVO, EV_TC_DPL_WIGGLE_SERVO},
    {MAV_CMD_CUT_DROGUE, EV_TC_CUT_DROGUE},

    {MAV_CMD_ARB_RESET_SERVO, EV_TC_ABK_RESET_SERVO},
    {MAV_CMD_ARB_WIGGLE_SERVO, EV_TC_ABK_WIGGLE_SERVO},
    {MAV_CMD_DISABLE_AEROBRAKES, EV_TC_ABK_DISABLE},
    {MAV_CMD_TEST_AEROBRAKES, EV_TC_TEST_ABK},

    {MAV_CMD_CALIBRATE_ALGOS, EV_TC_CALIBRATE_ALGOS},
    {MAV_CMD_CALIBRATE_SENSORS, EV_TC_CALIBRATE_SENSORS},

    {MAV_CMD_SERIAL_TM, EV_TC_SERIAL_TM},
    {MAV_CMD_RADIO_TM, EV_TC_RADIO_TM},

    {MAV_CMD_START_LOGGING, EV_TC_START_LOG},
    {MAV_CMD_CLOSE_LOG, EV_TC_CLOSE_LOG},

    {MAV_CMD_TEST_MODE, EV_TC_TEST_MODE},
    {MAV_CMD_BOARD_RESET, EV_TC_RESET_BOARD},
    {MAV_CMD_END_MISSION, EV_TC_END_MISSION}};

void handleMavlinkMessage(MavDriver* mav_driver, const mavlink_message_t& msg)
{
    // log status
    DeathStack::getInstance()->radio->logStatus();

    // acknowledge
    sendAck(mav_driver, msg);

    // handle TC
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_NOARG_TC:  // basic command
        {
            LOG_DEBUG(print_logger, "Received NOARG command");
            uint8_t commandId = mavlink_msg_noarg_tc_get_command_id(&msg);

            // search for the corresponding event and post it
            auto it = tcMap.find(commandId);
            if (it != tcMap.end())
                sEventBroker->post(Event{it->second}, TOPIC_TMTC);
            else
                LOG_WARN(print_logger, "Unknown NOARG command {:d}", commandId);

            switch (commandId)
            {
                case MAV_CMD_BOARD_RESET:
                    logger->stop();
                    LOG_INFO(print_logger, "Received command BOARD_RESET");
                    miosix::reboot();
                    break;
                case MAV_CMD_CLOSE_LOG:
                    logger->stop();
                    sendTelemetry(mav_driver, MAV_LOGGER_TM_ID);
                    LOG_INFO(print_logger, "Received command CLOSE_LOG");
                    break;
                case MAV_CMD_START_LOGGING:
                    DeathStack::getInstance()->startLogger();
                    sendTelemetry(mav_driver, MAV_LOGGER_TM_ID);
                    LOG_INFO(print_logger, "Received command START_LOG");
                    break;
                default:
                    break;
            }
            break;
        }

        case MAVLINK_MSG_ID_TELEMETRY_REQUEST_TC:  // tm request
        {
            uint8_t tmId = mavlink_msg_telemetry_request_tc_get_board_id(&msg);
            LOG_DEBUG(print_logger, "Received TM request : id = {:d}", tmId);

            // send corresponding telemetry or NACK
            sendTelemetry(mav_driver, tmId);

            break;
        }
        case MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE:
        {
            float alt =
                mavlink_msg_set_reference_altitude_get_ref_altitude(&msg);
            LOG_DEBUG(
                print_logger,
                "Received SET_REFERENCE_ALTITUDE command. Ref altitude: {:f} m",
                alt);
            DeathStack::getInstance()->state_machines->setReferenceAltitude(
                alt);
            break;
        }
        case MAVLINK_MSG_ID_SET_REFERENCE_TEMPERATURE_TC:
        {
            float temp =
                mavlink_msg_set_reference_temperature_tc_get_ref_temp(&msg);
            LOG_DEBUG(
                print_logger,
                "Received SET_REFERENCE_TEMPERATURE command. Temp: {:f} degC",
                temp);
            DeathStack::getInstance()->state_machines->setReferenceTemperature(
                temp);
            break;
        }
        case MAVLINK_MSG_ID_SET_DEPLOYMENT_ALTITUDE_TC:
        {
            float alt =
                mavlink_msg_set_deployment_altitude_tc_get_dpl_altitude(&msg);
            LOG_DEBUG(
                print_logger,
                "Received SET_DEPLOYMENT_ALTITUDE command. Dpl alt: {:f} m",
                alt);
            DeathStack::getInstance()
                ->state_machines->ada_controller->setDeploymentAltitude(alt);
            break;
        }
        case MAVLINK_MSG_ID_SET_INITIAL_ORIENTATION_TC:
        {
            float yaw = mavlink_msg_set_initial_orientation_tc_get_yaw(&msg);
            float pitch =
                mavlink_msg_set_initial_orientation_tc_get_pitch(&msg);
            float roll = mavlink_msg_set_initial_orientation_tc_get_roll(&msg);
            LOG_DEBUG(print_logger,
                      "Received SET_INITIAL_ORIENTATION command. roll: {:f}, "
                      "pitch: {:f}, yaw: {:f}",
                      roll, pitch, yaw);
            DeathStack::getInstance()->state_machines->setInitialOrientation(
                roll, pitch, yaw);
            break;
        }

        case MAVLINK_MSG_ID_SET_INITIAL_COORDINATES_TC:
        {
            float latitude =
                mavlink_msg_set_initial_coordinates_tc_get_latitude(&msg);
            float longitude =
                mavlink_msg_set_initial_coordinates_tc_get_longitude(&msg);
            LOG_DEBUG(
                print_logger,
                "Received SET_INITIAL_COORDINATES command. latitude: {:f}, "
                "longitude: {:f}",
                latitude, longitude);
            DeathStack::getInstance()->state_machines->setInitialCoordinates(
                latitude, longitude);
            break;
        }
        case MAVLINK_MSG_ID_RAW_EVENT_TC:  // post a raw event
        {
            LOG_DEBUG(print_logger, "Received RAW_EVENT command");

            // post given event on given topic
            sEventBroker->post({mavlink_msg_raw_event_tc_get_Event_id(&msg)},
                               mavlink_msg_raw_event_tc_get_Topic_id(&msg));
            break;
        }

        case MAVLINK_MSG_ID_PING_TC:
        {
            LOG_DEBUG(print_logger, "Ping received");
            break;
        }
        case MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC:
        {
            float pos = mavlink_msg_set_aerobrake_angle_tc_get_angle(&msg);
            DeathStack::getInstance()
                ->state_machines->arb_controller->setAirBrakesPosition(pos);
            LOG_DEBUG(print_logger, "Received set ab pos: {:.1f} deg", pos);

            break;
        }
        default:
        {
            LOG_DEBUG(print_logger, "Received message is not of a known type");
            break;
        }
    }
}
void sendAck(MavDriver* mav_driver, const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(TMTC_MAV_SYSID, TMTC_MAV_COMPID, &ackMsg, msg.msgid,
                            msg.seq);

    mav_driver->enqueueMsg(ackMsg);
    LOG_INFO(print_logger, "Enqueued Ack ({:d})", msg.seq);
}

bool sendTelemetry(MavDriver* mav_driver, const uint8_t tm_id)
{
    // enqueue the TM packet taking it from the TM repo (pauses kernel to
    // guarantee synchronicity)
    bool ok =
        mav_driver->enqueueMsg(TmRepository::getInstance()->packTM(tm_id));

    // update status
    DeathStack::getInstance()->radio->logStatus();

    return ok;
}

}  // namespace DeathStackBoard