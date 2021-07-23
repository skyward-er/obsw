/* Copyright (c) 2018 Skyward Experimental Rocketry
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

#include "IgnitionController.h"

#include "boards/CanInterfaces.h"
#include "configs/IgnitionConfig.h"
#include "events/Events.h"

namespace DeathStackBoard
{

using namespace CanInterfaces;

IgnitionController::IgnitionController(CanProxy* canbus)
    : FSM(&IgnitionController::stateIdle), canbus(canbus)
{
    // Receive self posted events
    sEventBroker->subscribe(this, TOPIC_IGNITION);
    // Receive global board events, e.g. liftoff
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    // Receive telecommands to be forwarded on the CAN
    sEventBroker->subscribe(this, TOPIC_TC);
    // Receive Can messages
    sEventBroker->subscribe(this, TOPIC_CAN);

    memset(&status, 0, sizeof(IgnCtrlStatus));
    memset(&loggable_board_status, 0, sizeof(IgnBoardLoggableStatus));
}

/**
 * Status
 */
void IgnitionController::logStatus()
{
    status.timestamp = miosix::getTick();
    logger.log(status);
}

bool IgnitionController::updateIgnBoardStatus(const Event& ev)
{
    const CanbusEvent& cev = static_cast<const CanbusEvent&>(ev);

    // Check event
    if (cev.canTopic == CAN_TOPIC_IGNITION &&
        cev.len == sizeof(IgnitionBoardStatus))
    {
        // Update internal board status struct
        memcpy(&(loggable_board_status.board_status), cev.payload,
               sizeof(IgnitionBoardStatus));

        // Log internal board status struct
        loggable_board_status.timestamp = miosix::getTick();
        logger.log(loggable_board_status);

        return true;
    }

    // Event not directed to me
    return false;
}

/**
 * States
 */
void IgnitionController::stateIdle(const Event& ev)
{
    status.last_event = ev.sig;

    switch (ev.sig)
    {
        case EV_ENTRY:
            status.fsm_state = IgnitionControllerState::IGN_IDLE;
            logStatus();

            // Schedule IGN_OFFLINE event: every time a message is received, the
            // event is rescheduled
            ign_offline_delayed_id = sEventBroker->postDelayed(
                {EV_IGN_OFFLINE}, TOPIC_FLIGHT_EVENTS, TIMEOUT_IGN_OFFLINE);

            // Send first getstatus event, which will be periodically
            // rescheduled
            sEventBroker->post({EV_IGN_GETSTATUS}, TOPIC_IGNITION);

            TRACE("IGNCTRL: Entering stateIdle\n");
            break;

        case EV_EXIT:
            // Remove GET_STATUS scheduled event
            sEventBroker->removeDelayed(get_status_delayed_id);
            TRACE("IGNCTRL: Exiting stateIdle\n");
            break;

        case EV_IGN_GETSTATUS:
        {
            status.n_sent_messages++;

            // Send status request on the CAN bus
            uint8_t cmd = CAN_MSG_REQ_IGN_STATUS;
            canbus->send(CAN_TOPIC_HOMEONE, &cmd, sizeof(uint8_t));

            // Post next GETSTATUS event
            get_status_delayed_id = sEventBroker->postDelayed(
                {EV_IGN_GETSTATUS}, TOPIC_IGNITION, INTERVAL_IGN_GET_STATUS);
            break;
        }

        case EV_NEW_CAN_MSG:
        {
            // Check that the event is an ignition status: if so, update
            // internal status
            if (updateIgnBoardStatus(ev))
            {
                status.n_rcv_messages++;

                // Reset the ignition offline timeout
                sEventBroker->removeDelayed(ign_offline_delayed_id);
                // Reschedule offline event
                ign_offline_delayed_id = sEventBroker->postDelayed(
                    {EV_IGN_OFFLINE}, TOPIC_FLIGHT_EVENTS, TIMEOUT_IGN_OFFLINE);

                static const uint16_t ABORT_BITMASK = 0x0707;
                uint16_t status_bytes;
                memcpy(&status_bytes, &loggable_board_status.board_status,
                       sizeof(IgnitionBoardStatus));
                if (status_bytes & ABORT_BITMASK)
                {
                    // We've had an abort.
                    status.abort_rcv = 1;
                    transition(&IgnitionController::stateAborted);
                }

                // Log ignition controller status
                logStatus();
            }
            break;
        }

        case EV_LIFTOFF:
            transition(&IgnitionController::stateEnd);
            break;

        case EV_GS_OFFLINE:
        case EV_TC_ABORT_LAUNCH:
        {
            status.n_sent_messages++;
            status.abort_sent = 1;

            // Send an ABORT message to the board
            // NOTE: this does not cause the controller to go in abort state
            // yet: this state wll be reached only when we have confirmation of
            // the ABORT from the board
            uint8_t cmd = CAN_MSG_ABORT;
            canbus->send((uint16_t)CAN_TOPIC_HOMEONE, &cmd, sizeof(uint8_t));
            break;
        }

        case EV_LAUNCH:
        {
            // Send an LAUNCH command to the board
            // NOTE: this does not cause the controller to change state: the
            // state will change only when we have confirmation from the board
            // and the detachment pin signaled the LIFTOFF event.
            const LaunchEvent& lev = static_cast<const LaunchEvent&>(ev);
            status.n_sent_messages++;
            status.launch_sent = 1;

            canbus->send((uint16_t)CAN_TOPIC_LAUNCH,
                         (uint8_t*)&(lev.launchCode), sizeof(uint64_t));
            break;
        }

        default:
            TRACE("IGNCTRL stateIdle: Event %d not handled.\n", ev.sig);
            break;
    }
}

void IgnitionController::stateAborted(const Event& ev)
{
    status.last_event = ev.sig;

    switch (ev.sig)
    {
        case EV_ENTRY:
            status.fsm_state = IgnitionControllerState::IGN_ABORTED;
            logStatus();

            // Send first getstatus event, which will be periodically
            // rescheduled
            sEventBroker->post({EV_IGN_GETSTATUS}, TOPIC_IGNITION);
            // Signal abort to rest of the board
            sEventBroker->post({EV_IGN_ABORTED}, TOPIC_FLIGHT_EVENTS);
            TRACE("IGNCTRL: Entering stateAborted\n");
            break;

        case EV_EXIT:
            TRACE("IGNCTRL: Exiting stateAborted\n");
            break;

        case EV_IGN_GETSTATUS:
        {
            status.n_sent_messages++;

            // Send status request on the CAN bus
            uint8_t cmd = CAN_MSG_REQ_IGN_STATUS;
            canbus->send(CAN_TOPIC_HOMEONE, &cmd, sizeof(uint8_t));

            // Post next GETSTATUS event
            get_status_delayed_id = sEventBroker->postDelayed(
                {EV_IGN_GETSTATUS}, TOPIC_IGNITION, INTERVAL_IGN_GET_STATUS);
            break;
        }

        // Still handle the abort, just in case we want to send it again
        case EV_TC_ABORT_LAUNCH:
        {
            status.n_sent_messages++;
            status.abort_sent = 1;

            uint8_t cmd = CAN_MSG_ABORT;
            canbus->send((uint16_t)CAN_TOPIC_HOMEONE, &cmd, sizeof(uint8_t));
            break;
        }

        case EV_NEW_CAN_MSG:
        {
            if (updateIgnBoardStatus(ev))
            {
                status.n_rcv_messages++;

                // Reset the ignition offline timeout
                sEventBroker->removeDelayed(ign_offline_delayed_id);
                // Reschedule offline event
                ign_offline_delayed_id = sEventBroker->postDelayed(
                    {EV_IGN_OFFLINE}, TOPIC_FLIGHT_EVENTS, TIMEOUT_IGN_OFFLINE);

                logStatus();
            }
            break;
        }

        default:
            TRACE("IGNCTRL stateAborted: Event %d not handled.\n", ev.sig);
            break;
    }
}

void IgnitionController::stateEnd(const Event& ev)
{
    status.last_event = ev.sig;

    switch (ev.sig)
    {
        case EV_ENTRY:
            sEventBroker->removeDelayed(ign_offline_delayed_id);
            status.fsm_state = IgnitionControllerState::IGN_END;
            logStatus();
            TRACE("IGNCTRL: Entering stateEnd\n");
            break;

        case EV_EXIT:
            TRACE("IGNCTRL: Exiting stateEnd\n");
            break;

            // No event handled here

        default:
            TRACE("IGNCTRL stateEnd: Event %d not handled.\n", ev.sig);
            break;
    }
}

}  // namespace DeathStackBoard
