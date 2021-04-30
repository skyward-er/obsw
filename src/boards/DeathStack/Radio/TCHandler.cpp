/**
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta (luca.erbetta@skywarder.eu)
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "TCHandler.h"

#include <map>

#include "DeathStack.h"
#include "LoggerService/LoggerService.h"
#include "configs/TMTCConfig.h"
#include "events/Events.h"
#include "AeroBrakesController/WindData.h"

using std::map;
namespace DeathStackBoard
{

LoggerService* logger = LoggerService::getInstance();

bool sendTelemetry(MavDriver* mav_driver, const uint8_t tm_id);

const std::map<uint8_t, uint8_t> tcMap = {
    {MAV_CMD_ARM, EV_TC_ARM},
    {MAV_CMD_DISARM, EV_TC_DISARM},
    {MAV_CMD_CALIBRATE_ADA, EV_TC_CALIBRATE_ADA},

    {MAV_CMD_FORCE_INIT, EV_TC_FORCE_INIT},
    {MAV_CMD_FORCE_LAUNCH, EV_TC_LAUNCH},
    {MAV_CMD_NOSECONE_OPEN, EV_TC_NC_OPEN},
    {MAV_CMD_RESET_SERVO, EV_TC_RESET_SERVO},
    {MAV_CMD_WIGGLE_SERVO, EV_TC_WIGGLE_SERVO},

    {MAV_CMD_START_LOGGING, EV_TC_START_SENSOR_LOGGING},
    {MAV_CMD_STOP_LOGGING, EV_TC_STOP_SENSOR_LOGGING},
    {MAV_CMD_CLOSE_LOG, EV_TC_CLOSE_LOG},

    {MAV_CMD_TEST_MODE, EV_TC_TEST_MODE},
    {MAV_CMD_TEST_PRIMARY_CUTTER, EV_TC_TEST_CUTTER_PRIMARY},
    {MAV_CMD_TEST_BACKUP_CUTTER, EV_TC_TEST_CUTTER_BACKUP},
    {MAV_CMD_CUT_PRIMARY, EV_TC_CUT_PRIMARY},
    {MAV_CMD_CUT_BACKUP, EV_TC_CUT_BACKUP},
    {MAV_CMD_CUT_DROGUE, EV_TC_CUT_DROGUE},
    {MAV_CMD_BOARD_RESET, EV_TC_BOARD_RESET},

    {MAV_CMD_END_MISSION, EV_TC_END_MISSION}};

void handleMavlinkMessage(MavDriver* mav_driver, const mavlink_message_t& msg)
{
    TRACE("[TMTC] Handling command\n");

    // log status
    logger->log(mav_driver->getStatus());

    // acknowledge
    sendAck(mav_driver, msg);

    // handle TC
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_NOARG_TC:  // basic command
        {
            TRACE("[TMTC] Received NOARG command\n");
            uint8_t commandId = mavlink_msg_noarg_tc_get_command_id(&msg);

            // search for the corresponding event and post it
            auto it = tcMap.find(commandId);
            if (it != tcMap.end())
                sEventBroker->post(Event{it->second}, TOPIC_TMTC);
            else
                TRACE("[TMTC] Unknown NOARG command %d\n", commandId);
            
            switch (commandId)
            {
            case MAV_CMD_BOARD_RESET:
                logger->stop();

                miosix::reboot();
                break;
            case MAV_CMD_CLOSE_LOG:
                logger->stop();
                break;
            default:
                break;
            }
            break;
        }

        case MAVLINK_MSG_ID_TELEMETRY_REQUEST_TC:  // tm request
        {
            TRACE("[TMTC] Received TM request\n");
            uint8_t tmId = mavlink_msg_telemetry_request_tc_get_board_id(&msg);

            // send corresponding telemetry or NACK
            sendTelemetry(mav_driver, tmId);

            break;
        }

        case MAVLINK_MSG_ID_UPLOAD_SETTING_TC:  // set a configuration parameter
        {
            // uint8_t id    =
            // mavlink_msg_upload_setting_tc_get_setting_id(&msg); float setting
            // = mavlink_msg_upload_setting_tc_get_setting(&msg);

            // TRACE("[TMTC] Upload setting: %d, %f\n", (int)id, setting);

            // // modify correspondig setting
            // switch (id)
            // {
            //     case MAV_SET_DEPLOYMENT_ALTITUDE:
            //     {
            //         ada.setDeploymentAltitude(setting);
            //         break;
            //     }
            //     case MAV_SET_REFERENCE_ALTITUDE:
            //     {
            //         ada.setReferenceAltitude(setting);
            //         break;
            //     }
            //     case MAV_SET_REFERENCE_TEMP:
            //     {
            //         ada.setReferenceTemperature(setting);
            //         break;
            //     }
            // }
            break;
        }

        case MAVLINK_MSG_ID_RAW_EVENT_TC:  // post a raw event
        {
            TRACE("[TMTC] Received RAW_EVENT command\n");

            // post given event on given topic
            sEventBroker->post({mavlink_msg_raw_event_tc_get_Event_id(&msg)},
                               mavlink_msg_raw_event_tc_get_Topic_id(&msg));
            break;
        }

        case MAVLINK_MSG_ID_PING_TC:
        {
            TRACE("[TMTC] Ping received\n");
            break;
        }
        case MAVLINK_MSG_ID_SET_AEROBRAKE_ANGLE_TC:
        {
            float pos = mavlink_msg_set_aerobrake_angle_tc_get_angle(&msg);
            DeathStack::getInstance()->actuators->aerobrakes->set(pos);
            TRACE("[TMTC] Received set ab pos: %.1f deg\n", pos);

            break;
        }
        case MAVLINK_MSG_ID_SET_WIND_TUNNEL_WIND_SPEED:
        {
            WindData d;
            d.wind = mavlink_msg_set_wind_tunnel_wind_speed_get_wind_speed(&msg);
            d.timestamp = miosix::getTick();
            logger->log(d);
            TRACE("[TMTC] Received wind data: %.2f m/s\n", d.wind);
            break;
        }
        default:
        {
            TRACE("[TMTC] Received message is not of a known type\n");
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
    TRACE("[TMTC] Enqueued Ack (%d)\n", msg.seq);
}

bool sendTelemetry(MavDriver* mav_driver, const uint8_t tm_id)
{
    // enqueue the TM packet taking it from the TM repo (pauses kernel to
    // guarantee synchronicity)
    bool ok =
        mav_driver->enqueueMsg(TmRepository::getInstance()->packTM(tm_id));

    // update status
    logger->log(mav_driver->getStatus());

    return ok;
}

}  // namespace DeathStackBoard