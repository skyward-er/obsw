/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise De Faveri
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

#pragma once

#include <Common.h>

#include <DeathStack/LogProxy/LogProxy.h>
#include <DeathStack/configs/TMTCConfig.h>
#include "DeathStack/DeathStack.h"
#include "DeathStack/Events.h"
#include "DeathStack/Topics.h"
#include "TMBuilder.h"

#define MAV_TC(X) MAVLINK_MSG_ID_##X##_TC

namespace DeathStackBoard
{

uint16_t g_gsOfflineEvId;
LoggerProxy& logger = *(LoggerProxy::getInstance());

namespace TCHandler
{

/**
 * Map each noArg command to the corresponding event
 */
// clang-format off
static const std::map<uint8_t, uint8_t> noargCmdToEvt = 
{
    { MAV_CMD_ARM,              EV_TC_ARM    }, 
    { MAV_CMD_DISARM,           EV_TC_DISARM }, 
    { MAV_CMD_CALIBRATE_ADA,    EV_TC_CALIBRATE_ADA},
    // { MAV_CMD_ABORT_LAUNCH, EV_TC_ABORT_LAUNCH},
    { MAV_CMD_ABORT_ROGALLO,    EV_TC_ABORT_ROGALLO},
    { MAV_CMD_FORCE_INIT,       EV_TC_FORCE_INIT},

    { MAV_CMD_NOSECONE_OPEN,    EV_TC_NC_OPEN }, 
    { MAV_CMD_NOSECONE_CLOSE,   EV_TC_NC_CLOSE }, 
    { MAV_CMD_CUT_MAIN,         EV_TC_CUT_MAIN },
    { MAV_CMD_CUT_DROGUE,       EV_TC_CUT_FIRST_DROGUE },
    { MAV_CMD_START_ROGALLO_CONTROL, EV_TC_START_ROGALLO_CONTROL},

    { MAV_CMD_START_LOGGING,    EV_TC_START_SENSOR_LOGGING }, 
    { MAV_CMD_STOP_LOGGING,     EV_TC_STOP_SENSOR_LOGGING }, 
    { MAV_CMD_CLOSE_LOG,        EV_TC_CLOSE_LOG }, 

    { MAV_CMD_TEST_MODE,        EV_TC_TEST_MODE  }, 
    { MAV_CMD_BOARD_RESET,      EV_TC_BOARD_RESET }, 
    { MAV_CMD_MANUAL_MODE,      EV_TC_MANUAL_MODE },

    { MAV_CMD_END_MISSION,      EV_TC_END_MISSION }
};
// clang-format on

/**
 * Send an ACK to notify the sender that you received the given message.
 */
static void sendAck(MavChannel* channel, const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(TMTC_MAV_SYSID, TMTC_MAV_COMPID, &ackMsg, msg.msgid,
                            msg.seq);

    /* Send the message back to the sender */
    channel->enqueueMsg(ackMsg);
    TRACE("[TMTC] Enqueued Ack\n");
}

/**
 *  Handle the Mavlink message, posting the corresponding event if needed.
 */
static void handleMavlinkMessage(MavChannel* channel,
                                 const mavlink_message_t& msg)
{
    TRACE("[TMTC] Handling command\n");

    /* Log Status */
    MavStatus status = channel->getStatus();
    logger.log(status);

    /* Send acknowledge */
    sendAck(channel, msg);

    /* Finally handle TC */
    switch (msg.msgid)
    {
        case MAV_TC(NOARG):
        {
            TRACE("[TMTC] Received NOARG command\n");
            uint8_t commandId = mavlink_msg_noarg_tc_get_command_id(&msg);
            auto it = DeathStackBoard::TCHandler::noargCmdToEvt.find(commandId);

            if (it != noargCmdToEvt.end())
                sEventBroker->post(Event{it->second}, TOPIC_TC);
            else
                TRACE("[TMTC] Unkown NOARG command %d\n", commandId);

            break;
        }

        case MAV_TC(TELEMETRY_REQUEST):
        {
            TRACE("[TMTC] Received TM request\n");
            uint8_t tmId = mavlink_msg_telemetry_request_tc_get_board_id(&msg);
            mavlink_message_t response = TMBuilder::buildTelemetry(tmId);
            channel->enqueueMsg(response);

            break;
        }

        case MAV_TC(PING):
        {
            TRACE("[TMTC] Ping received\n");
            break;
        }
        case MAV_TC(UPLOAD_SETTING):
        {
            uint8_t id    = mavlink_msg_upload_setting_tc_get_setting_id(&msg);
            float setting = mavlink_msg_upload_setting_tc_get_setting(&msg);

            switch (id)
            {
                case MAV_SET_DEPLOYMENT_ALTITUDE:
                {
                    DeathStack::getInstance()->ada->setDeploymentAltitude(
                        setting);
                    break;
                }
                case MAV_SET_REFERENCE_ALTITUDE:
                {
                    DeathStack::getInstance()->ada->setReferenceAltitude(
                        setting);
                    break;
                }
                case MAV_SET_REFERENCE_TEMP:
                {
                    DeathStack::getInstance()->ada->setReferenceTemperature(
                        setting);
                    break;
                }
            }
            break;
        }
        case MAV_TC(RAW_EVENT):
        {
            TRACE("[TMTC] Received RAW_EVENT command\n");
            /* Retrieve event from the message*/
            Event evt = {mavlink_msg_raw_event_tc_get_Event_id(&msg)};
            sEventBroker->post(evt,
                               mavlink_msg_raw_event_tc_get_Topic_id(&msg));
            break;
        }

        default:
        {
            TRACE("[TMTC] Received message is not of a known type\n");
            // TODO: fault counter?
            break;
        }
    }
}

} /* namespace TCHandler */
} /* namespace DeathStackBoard */
