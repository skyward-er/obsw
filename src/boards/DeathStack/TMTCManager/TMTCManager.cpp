/* Copyright (c) 2018 Skyward Experimental Rocketry
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
#include <DeathStack/configs/TMTCConfig.h>
#include <DeathStack/events/Events.h>
#include <DeathStack/events/Topics.h>
#include <drivers/Xbee/Xbee.h>
#include "TCHandler.h"  // Real message handling is here
#include "XbeeInterrupt.h"
#include "bitpacking/hermes/HermesPackets.h"
namespace DeathStackBoard
{

TMTCManager::TMTCManager() : FSM(&TMTCManager::stateGroundTM)
{
    busSPI2::init();
    enableXbeeInterrupt();

    device  = new Xbee_t();
    channel = new Mav(device,
                      &TCHandler::handleMavlinkMessage,  // rcv function
                      TMTC_SLEEP_AFTER_SEND, MAV_OUT_BUFFER_MAX_AGE);

    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_TMTC);

    TRACE("[TMTC] Created TMTCManager\n");
}

TMTCManager::~TMTCManager()
{
    device->stop();
    sEventBroker->unsubscribe(this);

    channel->stop();
    delete device;
    delete channel;
}

bool TMTCManager::send(mavlink_message_t& msg)
{
    bool ok = channel->enqueueMsg(msg);

    MavlinkStatus status = channel->getStatus();
    logger.log(status);

    return ok;
}

void TMTCManager::stateGroundTM(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            test_tm_event_id = sEventBroker->postDelayed<TEST_TM_TIMEOUT>(
                Event{EV_SEND_TEST_TM}, TOPIC_TMTC);

            TRACE("[TMTC] Entering stateTestTM\n");
            StackLogger::getInstance()->updateStack(THID_TMTC_FSM);
            break;

        case EV_SEND_TEST_TM:
        {
            // Send both HR_TM and TEST_TM

            // Pack the current data in hr_tm_packet.payload
            packHRTelemetry(hr_tm_packet.payload, hr_tm_index);

            // Send HR telemetry once 4 packets are filled
            if (hr_tm_index == 3)
            {
                mavlink_message_t telem =
                    TMBuilder::buildTelemetry(MAV_HR_TM_ID);
                send(telem);
            }

            // Two TEST_TM every one HR_TM
            if (hr_tm_index % 2 == 1)
            {
                mavlink_message_t telem =
                    TMBuilder::buildTelemetry(MAV_TEST_TM_ID);
                send(telem);
            }
            hr_tm_index = (hr_tm_index + 1) % 4;

            test_tm_event_id = sEventBroker->postDelayed<TEST_TM_TIMEOUT>(
                Event{EV_SEND_TEST_TM}, TOPIC_TMTC);
            break;
        }
        case EV_ARMED:
        case EV_LIFTOFF:
        {
            transition(&TMTCManager::stateFlightTM);
            break;
        }

        case EV_EXIT:
        {
            sEventBroker->removeDelayed(test_tm_event_id);

            TRACE("[TMTC] Exiting stateTestTM\n");
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
            lr_event_id = sEventBroker->postDelayed<LR_TM_TIMEOUT>(
                Event{EV_SEND_LR_TM}, TOPIC_TMTC);
            hr_event_id = sEventBroker->postDelayed<HR_TM_TIMEOUT>(
                Event{EV_SEND_HR_TM}, TOPIC_TMTC);

            TRACE("[TMTC] Entering stateFlightTM\n");
            StackLogger::getInstance()->updateStack(THID_TMTC_FSM);
            break;

        case EV_SEND_HR_TM:
        {
            // Pack the current data in hr_tm_packet.payload
            packHRTelemetry(hr_tm_packet.payload, hr_tm_index);

            // Send HR telemetry once 4 packets are filled
            if (hr_tm_index == 3)
            {
                mavlink_msg_hr_tm_encode(TMTC_MAV_SYSID, TMTC_MAV_SYSID,
                                         &auto_telemetry_msg, &(hr_tm_packet));
                send(auto_telemetry_msg);

            }

            hr_tm_index = (hr_tm_index + 1) % 4;

            // Schedule the next HR telemetry
            hr_event_id = sEventBroker->postDelayed<HR_TM_TIMEOUT>(
                Event{EV_SEND_HR_TM}, TOPIC_TMTC);

            break;
        }

        case EV_SEND_LR_TM:
        {
            packLRTelemetry(lr_tm_packet.payload);

            mavlink_message_t telem = TMBuilder::buildTelemetry(MAV_LR_TM_ID);
            send(telem);

            // Schedule the next HR telemetry
            lr_event_id = sEventBroker->postDelayed<LR_TM_TIMEOUT>(
                Event{EV_SEND_LR_TM}, TOPIC_TMTC);
            break;
        }
        case EV_DISARMED:
        {
            transition(&TMTCManager::stateGroundTM);
            break;
        }
        case EV_EXIT:
        {
            sEventBroker->removeDelayed(lr_event_id);
            sEventBroker->removeDelayed(hr_event_id);

            TRACE("[TMTC] Exiting stateFlightTM\n");
            break;
        }
        default:
            break;
    }
}

void TMTCManager::packHRTelemetry(uint8_t* packet, unsigned int index)
{
    HighRateTMPacker packer(packet);

    packer.packTimestamp(miosix::getTick(), index);

    packer.packPressureAda(tm_repository.hr_tm.pressure_ada, index);
    packer.packPressureDigi(tm_repository.hr_tm.pressure_digi, index);

    packer.packMslAltitude(tm_repository.hr_tm.msl_altitude, index);
    packer.packAglAltitude(tm_repository.hr_tm.agl_altitude, index);

    packer.packVertSpeed(tm_repository.hr_tm.vert_speed, index);
    packer.packVertSpeed2(tm_repository.hr_tm.vert_speed_2, index);

    packer.packAccX(tm_repository.hr_tm.acc_x, index);
    packer.packAccY(tm_repository.hr_tm.acc_y, index);
    packer.packAccZ(tm_repository.hr_tm.acc_z, index);

    packer.packGyroX(tm_repository.hr_tm.gyro_x, index);
    packer.packGyroY(tm_repository.hr_tm.gyro_y, index);
    packer.packGyroZ(tm_repository.hr_tm.gyro_z, index);

    packer.packGpsLat(tm_repository.hr_tm.gps_lat, index);
    packer.packGpsLon(tm_repository.hr_tm.gps_lon, index);
    packer.packGpsAlt(tm_repository.hr_tm.gps_alt, index);
    packer.packGpsFix(tm_repository.hr_tm.gps_fix, index);

    packer.packTemperature(tm_repository.hr_tm.temperature, index);

    packer.packFmmState(tm_repository.hr_tm.fmm_state, index);
    packer.packDplState(tm_repository.hr_tm.dpl_state, index);

    packer.packPinLaunch(tm_repository.hr_tm.pin_launch, index);
    packer.packPinNosecone(tm_repository.hr_tm.pin_nosecone, index);
}

void TMTCManager::packLRTelemetry(uint8_t* packet)
{
    LowRateTMPacker packer(packet);

    packer.packLiftoffTs(tm_repository.lr_tm.liftoff_ts, 0);
    packer.packLiftoffMaxAccTs(tm_repository.lr_tm.liftoff_max_acc_ts, 0);
    packer.packLiftoffMaxAcc(tm_repository.lr_tm.liftoff_max_acc_ts, 0);

    packer.packMaxZspeedTs(tm_repository.lr_tm.max_zspeed_ts, 0);
    packer.packMaxZspeed(tm_repository.lr_tm.max_zspeed, 0);
    packer.packMaxSpeedAltitude(tm_repository.lr_tm.max_speed_altitude, 0);

    packer.packApogeeTs(tm_repository.lr_tm.apogee_ts, 0);
    packer.packNxpMinPressure(tm_repository.lr_tm.nxp_min_pressure, 0);
    packer.packHwMinPressure(tm_repository.lr_tm.hw_min_pressure, 0);
    packer.packKalmanMinPressure(tm_repository.lr_tm.kalman_min_pressure, 0);
    packer.packDigitalMinPressure(tm_repository.lr_tm.digital_min_pressure, 0);

    packer.packBaroMaxAltitutde(tm_repository.lr_tm.baro_max_altitutde, 0);
    packer.packGpsMaxAltitude(tm_repository.lr_tm.gps_max_altitude, 0);

    packer.packApogeeLat(tm_repository.lr_tm.apogee_lat, 0);
    packer.packApogeeLon(tm_repository.lr_tm.apogee_lon, 0);

    packer.packDrogueDplTs(tm_repository.lr_tm.drogue_dpl_ts, 0);
    packer.packDrogueDplMaxAcc(tm_repository.lr_tm.drogue_dpl_max_acc, 0);

    packer.packMainDplTs(tm_repository.lr_tm.main_dpl_ts, 0);
    packer.packMainDplAltitude(tm_repository.lr_tm.main_dpl_altitude, 0);
    packer.packMainDplZspeed(tm_repository.lr_tm.main_dpl_zspeed, 0);
    packer.packMainDplAcc(tm_repository.lr_tm.main_dpl_acc, 0);
}

} /* namespace DeathStackBoard */
