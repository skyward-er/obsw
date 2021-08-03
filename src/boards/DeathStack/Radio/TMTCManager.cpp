/* Copyright (c) 2018-2020 Skyward Experimental Rocketry
 * Authors: Alvise de' Faveri Tron
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

#include "TMTCManager.h"

#include <LoggerService/LoggerService.h>
#include <configs/TMTCConfig.h>
#include <drivers/Xbee/Xbee.h>
#include <drivers/mavlink/MavlinkDriver.h>
#include <events/Events.h>
#include <events/Topics.h>
// #include <diagnostic/PrintLogger.h>

#include <cassert>

namespace DeathStackBoard
{

TMTCManager::TMTCManager()
    : FSM(&TMTCManager::stateGroundTM, skywardStack(16 * 1024))
{
    // init FSM
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_TMTC);

    LOG_DEBUG(log, "Created TMTCManager");
}

TMTCManager::~TMTCManager() { sEventBroker->unsubscribe(this); }

bool TMTCManager::send(const uint8_t tm_id)
{
    MavDriver* mav_driver = DeathStack::getInstance()->radio->mav_driver;
    TmRepository* tm_repo = DeathStack::getInstance()->radio->tm_repo;
    // enqueue the TM packet taking it from the TM repo (pauses kernel to
    // guarantee synchronicity)
    bool ok = mav_driver->enqueueMsg(tm_repo->packTM(tm_id));
    // update status
    logger.log(mav_driver->getStatus());

    return ok;
}

void TMTCManager::sendSerialTelemetry()
{
    TmRepository* tm_repo = DeathStack::getInstance()->radio->tm_repo;
    uint8_t buf[256];
    mavlink_message_t msg = tm_repo->packTM(MAV_HR_TM_ID);
    uint16_t len          = mavlink_msg_to_send_buffer(buf, &msg);
    fwrite(buf, sizeof(uint8_t), len, stdout);
    fflush(stdout);
}

// State Handlers
void TMTCManager::stateGroundTM(const Event& ev)
{
    // TmRepository* tm_repo = DeathStack::getInstance()->radio->tm_repo;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            // add periodic events
            periodicHrEvId = sEventBroker->postDelayed<HR_TM_GROUND_TIMEOUT>(
                Event{EV_SEND_HR_TM}, TOPIC_TMTC);
            periodicSensEvId =
                sEventBroker->postDelayed<GROUND_SENS_TM_TIMEOUT>(
                    Event{EV_SEND_SENS_TM}, TOPIC_TMTC);

            LOG_DEBUG(log, "Entering stateGroundTM");

            // log stack usage
            StackLogger::getInstance()->updateStack(THID_TMTC_FSM);
            break;
        }
        case EV_EXIT:
        {
            // remove periodic events
            sEventBroker->removeDelayed(periodicHrEvId);
            sEventBroker->removeDelayed(periodicLrEvId);
            sEventBroker->removeDelayed(periodicSensEvId);

            LOG_DEBUG(log, "Exiting stateGroundTM");
            break;
        }
        case EV_SEND_HR_TM:
        {
            // repost periodic event
            periodicHrEvId = sEventBroker->postDelayed<HR_TM_GROUND_TIMEOUT>(
                Event{EV_SEND_HR_TM}, TOPIC_TMTC);

            send(MAV_HR_TM_ID);

            break;
        }
        case EV_SEND_SENS_TM:
        {
            // LOG_DEBUG(log, "Sending SENS_TM");
            periodicSensEvId =
                sEventBroker->postDelayed<GROUND_SENS_TM_TIMEOUT>(
                    Event{EV_SEND_SENS_TM}, TOPIC_TMTC);

            // send tm
            send(MAV_SENSORS_TM_ID);
            break;
        }
        case EV_TC_START_SENSOR_TM:
        {
            transition(&TMTCManager::stateSensorTM);
            break;
        }
        case EV_TC_SERIAL_TM:
        {
            transition(&TMTCManager::stateSerialDebugTM);
            break;
        }
        case EV_ARMED:
        case EV_LIFTOFF:
        {
            transition(&TMTCManager::stateFlightTM);
            break;
        }
        default:
            break;
    }
}

// State Handlers
void TMTCManager::stateSensorTM(const Event& ev)
{
    // TmRepository* tm_repo = DeathStack::getInstance()->radio->tm_repo;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            // add periodic events
            periodicSensEvId = sEventBroker->postDelayed<SENS_TM_TIMEOUT>(
                Event{EV_SEND_SENS_TM}, TOPIC_TMTC);

            LOG_DEBUG(log, "Entering stateSensorTM");

            // log stack usage
            StackLogger::getInstance()->updateStack(THID_TMTC_FSM);
            break;
        }
        case EV_EXIT:
        {
            // remove periodic events
            sEventBroker->removeDelayed(periodicSensEvId);

            LOG_DEBUG(log, "Exiting stateSensorTM");
            break;
        }
        case EV_SEND_SENS_TM:
        {
            // repost periodic event
            periodicSensEvId = sEventBroker->postDelayed<SENS_TM_TIMEOUT>(
                Event{EV_SEND_SENS_TM}, TOPIC_TMTC);

            send(MAV_SENSORS_TM_ID);

            break;
        }
        case EV_TC_STOP_SENSOR_TM:
        {
            transition(&TMTCManager::stateGroundTM);
            break;
        }
        case EV_ARMED:
        case EV_LIFTOFF:
        {
            transition(&TMTCManager::stateFlightTM);
            break;
        }
        default:
            break;
    }
}

void TMTCManager::stateFlightTM(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            // add periodic events
            periodicLrEvId = sEventBroker->postDelayed<LR_TM_TIMEOUT>(
                Event{EV_SEND_LR_TM}, TOPIC_TMTC);
            periodicHrEvId = sEventBroker->postDelayed<HR_TM_TIMEOUT>(
                Event{EV_SEND_HR_TM}, TOPIC_TMTC);

            LOG_DEBUG(log, "Entering stateFlightTM");

            // log stack usage
            StackLogger::getInstance()->updateStack(THID_TMTC_FSM);
            break;
        }

        case EV_EXIT:
        {
            // remove periodic events
            sEventBroker->removeDelayed(periodicLrEvId);
            sEventBroker->removeDelayed(periodicHrEvId);

            LOG_DEBUG(log, "Exiting stateFlightTM");
            break;
        }
        case EV_SEND_HR_TM:
        {
            // repost periodic event
            periodicHrEvId = sEventBroker->postDelayed<HR_TM_TIMEOUT>(
                Event{EV_SEND_HR_TM}, TOPIC_TMTC);

            // send tm once 4 packets are filled
            send(MAV_HR_TM_ID);
            // bool full = tm_repo->updateHR();
            // if (full)
            // {
            //     send(MAV_HR_TM_ID);
            // }
            break;
        }
        case EV_SEND_LR_TM:
        {
            // repost periodic event
            periodicLrEvId = sEventBroker->postDelayed<LR_TM_TIMEOUT>(
                Event{EV_SEND_LR_TM}, TOPIC_TMTC);

            // send low rate tm
            send(MAV_LR_TM_ID);

            break;
        }
        case EV_DISARMED:
        {
            transition(&TMTCManager::stateGroundTM);
            break;
        }

        default:
            break;
    }
}

void TMTCManager::stateSerialDebugTM(const Event& ev)
{
    // TmRepository* tm_repo = DeathStack::getInstance()->radio->tm_repo;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            // add periodic events
            periodicHrEvId = sEventBroker->postDelayed<HR_TM_GROUND_TIMEOUT>(
                Event{EV_SEND_HR_TM_OVER_SERIAL}, TOPIC_TMTC);

            LOG_DEBUG(log, "Entering stateSerialDebugTM");

            // log stack usage
            StackLogger::getInstance()->updateStack(THID_TMTC_FSM);
            break;
        }
        case EV_EXIT:
        {
            // remove periodic events
            sEventBroker->removeDelayed(periodicHrEvId);

            LOG_DEBUG(log, "Exiting stateSerialDebugTM");
            break;
        }
        case EV_SEND_HR_TM_OVER_SERIAL:
        {
            // repost periodic event
            periodicHrEvId = sEventBroker->postDelayed<HR_TM_GROUND_TIMEOUT>(
                Event{EV_SEND_HR_TM_OVER_SERIAL}, TOPIC_TMTC);

            sendSerialTelemetry();

            break;
        }
        case EV_TC_SERIAL_TM:
        {
            transition(&TMTCManager::stateGroundTM);
            break;
        }
        default:
            break;
    }
}

// void TMTCManager::stateWindTunnelTM(const Event& ev)
// {
//     switch (ev.sig)
//     {
//         // case EV_ENTRY:
//         // {
//         //     // add periodic events
//         //     periodicTunnelEvId =
//         sEventBroker->postDelayed<TUNNEL_TM_TIMEOUT>(
//         //         Event{EV_SEND_TUNNEL_TM}, TOPIC_TMTC);

//         //     LOG_DEBUG(log, "Entering stateWindTunnelTM");

//         //     // log stack usage
//         //     StackLogger::getInstance()->updateStack(THID_TMTC_FSM);
//         //     break;
//         // }

//         // case EV_EXIT:
//         // {
//         //     // remove periodic events
//         //     sEventBroker->removeDelayed(periodicLrEvId);
//         //     sEventBroker->removeDelayed(periodicHrEvId);
//         //     sEventBroker->removeDelayed(periodicTunnelEvId);

//         //     LOG_DEBUG(log, "Exiting stateWindTunnelTM");
//         //     break;
//         // }

//         // case EV_SEND_TUNNEL_TM:
//         // {
//         //     // LOG_DEBUG(log, "EV_SEND stateWindTunnelTM");
//         //     // repost periodic event
//         //     periodicTunnelEvId =
//         sEventBroker->postDelayed<TUNNEL_TM_TIMEOUT>(
//         //         Event{EV_SEND_TUNNEL_TM}, TOPIC_TMTC);

//         //     // send tm once 4 packets are filled
//         //     send(MAV_WINDTUNNEL_TM_ID);
//         //     break;
//         // }

//         default:
//             break;
//     }
// }

} /* namespace DeathStackBoard */
