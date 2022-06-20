/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <Payload/Payload.h>
#include <Payload/TelemetriesTelecommands/TMRepository.h>

using namespace Boardcore;
namespace Payload
{
mavlink_message_t TMRepository::packSystemTM(uint8_t reqTm, uint8_t sysId,
                                             uint8_t compId)
{
    mavlink_message_t message;
    mavlink_nack_tm_t nack;

    // Pause the kernel lock to perform this operation

    switch (reqTm)
    {
        case SystemTMList::MAV_SYS_ID:
        {
            // Represents the PayloadStatus class
            PayloadStatus status = Payload::Payload::getInstance().status;

            // Update the repository
            tmRepository.sysTm.timestamp      = miosix::getTick();
            tmRepository.sysTm.death_stack    = status.payload;
            tmRepository.sysTm.logger         = status.logger;
            tmRepository.sysTm.ev_broker      = status.eventBroker;
            tmRepository.sysTm.pin_obs        = status.pinOBS;
            tmRepository.sysTm.radio          = status.radio;
            tmRepository.sysTm.state_machines = status.FMM;
            tmRepository.sysTm.sensors        = status.sensors;

            // Encode the message
            mavlink_msg_sys_tm_encode(sysId, compId, &message,
                                      &(tmRepository.sysTm));

            break;
        }
        case SystemTMList::MAV_FSM_ID:
        {
            break;
        }
        case SystemTMList::MAV_PIN_OBS_ID:
        {
            break;
        }
        case SystemTMList::MAV_LOGGER_ID:
        {
            // Get the logger stats
            Boardcore::LoggerStats stats =
                Payload::Payload::getInstance().SDlogger->getStats();

            // Update the repository
            tmRepository.loggerTm.timestamp = miosix::getTick();
            tmRepository.loggerTm.log_number =
                Payload::Payload::getInstance().SDlogger->getCurrentLogNumber();
            tmRepository.loggerTm.too_large_samples  = stats.tooLargeSamples;
            tmRepository.loggerTm.sdropped_samples   = stats.droppedSamples;
            tmRepository.loggerTm.queued_samples     = stats.queuedSamples;
            tmRepository.loggerTm.filled_buffers     = stats.buffersFilled;
            tmRepository.loggerTm.written_buffers    = stats.buffersWritten;
            tmRepository.loggerTm.failed_writes      = stats.writesFailed;
            tmRepository.loggerTm.error_writes       = stats.lastWriteError;
            tmRepository.loggerTm.average_write_time = stats.averageWriteTime;
            tmRepository.loggerTm.max_write_time     = stats.maxWriteTime;

            // Encode the message
            mavlink_msg_logger_tm_encode(sysId, compId, &message,
                                         &(tmRepository.loggerTm));
        }
        case SystemTMList::MAV_MAVLINK_STATS:
        {
            // Get the mavlink driver status
            MavlinkStatus status =
                Payload::Payload::getInstance().radio->mavDriver->getStatus();

            // Update the repository
            tmRepository.mavlinkStatsTm.timestamp      = miosix::getTick();
            tmRepository.mavlinkStatsTm.n_send_queue   = status.nSendQueue;
            tmRepository.mavlinkStatsTm.max_send_queue = status.maxSendQueue;
            tmRepository.mavlinkStatsTm.n_send_errors  = status.nSendErrors;
            tmRepository.mavlinkStatsTm.msg_received =
                status.mavStats.msg_received;
            tmRepository.mavlinkStatsTm.buffer_overrun =
                status.mavStats.buffer_overrun;
            tmRepository.mavlinkStatsTm.parse_error =
                status.mavStats.parse_error;
            tmRepository.mavlinkStatsTm.parse_state =
                status.mavStats.parse_state;
            tmRepository.mavlinkStatsTm.packet_idx = status.mavStats.packet_idx;
            tmRepository.mavlinkStatsTm.current_rx_seq =
                status.mavStats.current_rx_seq;
            tmRepository.mavlinkStatsTm.current_tx_seq =
                status.mavStats.current_tx_seq;
            tmRepository.mavlinkStatsTm.packet_rx_success_count =
                status.mavStats.packet_rx_success_count;
            tmRepository.mavlinkStatsTm.packet_rx_drop_count =
                status.mavStats.packet_rx_drop_count;

            // Encode the message
            mavlink_msg_mavlink_stats_tm_encode(sysId, compId, &message,
                                                &(tmRepository.mavlinkStatsTm));

            break;
        }
        case SystemTMList::MAV_TASK_STATS_ID:
        {
            // TODO NEED TO DISCUSS
            break;
        }
        case SystemTMList::MAV_DPL_ID:
        {
            break;
        }
        case SystemTMList::MAV_ADA_ID:
        {
            break;
        }
        case SystemTMList::MAV_NAS_ID:
        {
            break;
        }
        case SystemTMList::MAV_CAN_ID:
        {
            break;
        }
        case SystemTMList::MAV_FLIGHT_ID:
        {
            break;
        }
        case SystemTMList::MAV_FLIGHT_STATS_ID:
        {
            break;
        }
        case SystemTMList::MAV_SENSORS_STATE_ID:
        {
            // TODO SAME THING AS MAV_TASK_STATS
            break;
        }
        default:
        {
            // Send the nack
            LOG_DEBUG(logger, "Unknown telemetry id: {:d}", reqTm);
            nack.recv_msgid = 0;
            nack.seq_ack    = 0;
            mavlink_msg_nack_tm_encode(sysId, compId, &message, &nack);
        }
    }
    return message;
}
mavlink_message_t TMRepository::packSensorTM(uint8_t reqTm, uint8_t sysId,
                                             uint8_t compId)
{
    mavlink_message_t message;
    return message;
}

}  // namespace Payload