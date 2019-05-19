/**
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#include "FlightStats.h"
#include <cmath>
#include "DeathStack/Events.h"
#include "LogProxy.h"
#include "events/EventBroker.h"

namespace DeathStackBoard
{

FlightStats::FlightStats() : FSM(&FlightStats::state_idle)
{
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_DEPLOYMENT);
}

FlightStats::~FlightStats() { sEventBroker->unsubscribe(this); }

void FlightStats::update(const KalmanState& t)
{
    switch (state)
    {
        case State::IDLE:
        {
            break;
        }
        case State::LIFTOFF:
        {
            break;
        }
        case State::ASCENDING:
        {
            if (t.x0 < apogee_stats.kalman_min_pressure)
            {
                apogee_stats.kalman_min_pressure = t.x0;
            }
            break;
        }
        case State::DROGUE_DPL:
        {
            break;
        }
        case State::MAIN_DPL:
        {
            break;
        }
    }
}
void FlightStats::update(const KalmanAltitude& t)
{
    switch (state)
    {
        case State::IDLE:
        {
            break;
        }
        case State::LIFTOFF:
        {
            if (t.vert_speed > liftoff_stats.vert_speed_max)
            {
                liftoff_stats.vert_speed_max = t.vert_speed;
                liftoff_stats.T_max_speed =
                    static_cast<uint32_t>(miosix::getTick());
                liftoff_stats.altitude_max_speed = t.altitude;
            }
            break;
        }
        case State::ASCENDING:
        {
            if (t.altitude > apogee_stats.baro_max_altitude)
            {
                apogee_stats.baro_max_altitude = t.altitude;
            }
            break;
        }
        case State::DROGUE_DPL:
        {
            break;
        }
        case State::MAIN_DPL:
        {
            // Only set it one time
            if (main_dpl_stats.altitude_dpl == 0)
            {
                main_dpl_stats.altitude_dpl   = t.altitude;
                main_dpl_stats.vert_speed_dpl = t.vert_speed;
            }
            break;
        }
    }
}
void FlightStats::update(const AD7994WrapperData& t)
{
    switch (state)
    {
        case State::IDLE:
        {
            break;
        }
        case State::LIFTOFF:
        {
            break;
        }
        case State::ASCENDING:
        {
            if (t.nxp_baro_pressure < apogee_stats.nxp_min_pressure)
            {
                apogee_stats.nxp_min_pressure = t.nxp_baro_pressure;
            }
            if (t.honeywell_baro_pressure < apogee_stats.hw_min_pressure)
            {
                apogee_stats.hw_min_pressure = t.honeywell_baro_pressure;
            }
            break;
        }
        case State::DROGUE_DPL:
        {
            break;
        }
        case State::MAIN_DPL:
        {
            break;
        }
    }
}
void FlightStats::update(const MPU9250Data& t)
{
    switch (state)
    {
        case State::IDLE:
        {
            break;
        }
        case State::LIFTOFF:
        {
            if (fabs(t.accel.getZ()) > liftoff_stats.acc_max)
            {
                liftoff_stats.T_max_acc =
                    static_cast<uint32_t>(miosix::getTick());
                liftoff_stats.acc_max = fabs(t.accel.getZ());
            }
            break;
        }
        case State::ASCENDING:
        {
            break;
        }
        case State::DROGUE_DPL:
        {
            if (fabs(t.accel.getZ()) > drogue_dpl_stats.max_dpl_acc)
            {
                drogue_dpl_stats.max_dpl_acc = fabs(t.accel.getZ());
                drogue_dpl_stats.T_dpl =
                    static_cast<uint32_t>(miosix::getTick());
            }
            break;
        }
        case State::MAIN_DPL:
        {
            if (fabs(t.accel.getZ()) > main_dpl_stats.max_dpl_acc)
            {
                main_dpl_stats.max_dpl_acc = fabs(t.accel.getZ());
            }
            break;
        }
    }
}

void FlightStats::update(const PiksiData& t)
{
    switch (state)
    {
        case State::IDLE:
        {
            break;
        }
        case State::LIFTOFF:
        {
            break;
        }
        case State::ASCENDING:
        {
            if (t.gps_data.height > apogee_stats.gps_max_altitude)
            {
                apogee_stats.gps_max_altitude = t.gps_data.height;
                apogee_stats.lat_apogee       = static_cast<float>(t.gps_data.latitude);
                apogee_stats.lon_apogee       = static_cast<float>(t.gps_data.longitude);
            }
            break;
        }
        case State::DROGUE_DPL:
        {
            break;
        }
        case State::MAIN_DPL:
        {
            break;
        }
    }
}

void FlightStats::state_idle(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("[FlightStats] Entering IDLE state\n");
            state = State::IDLE;
            try
            {
                while (!deferred_events.isEmpty())
                {
                    postEvent(deferred_events.pop());
                }
            }
            catch (...)
            {
                TRACE("Tried to pop empty circularbuffer!\n");
            }
            break;
        }
        case EV_EXIT:
        {
            TRACE("[FlightStats] Exiting IDLE state\n");

            break;
        }
        case EV_LIFTOFF:
        {
            transition(&FlightStats::state_liftOff);
            break;
        }
        case EV_DPL_ALTITUDE:
        {
            transition(&FlightStats::state_mainDeployment);
            break;
        }
        default:
        {
            break;
        }
    }
}
void FlightStats::state_liftOff(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("[FlightStats] Entering LIFTOFF state\n");
            state = State::LIFTOFF;

            ev_timeout_id = sEventBroker->postDelayed(
                {EV_STATS_TIMEOUT}, TOPIC_STATS,
                FlightStatsConfig::TIMEOUT_STATE_LIFTOFF);

            liftoff_stats.T_liftoff = static_cast<uint32_t>(miosix::getTick());
            break;
        }
        case EV_EXIT:
        {
            TRACE("[FlightStats] Exiting LIFTOFF state\n");

            LoggerProxy::getInstance()->log(liftoff_stats);
            sEventBroker->removeDelayed(ev_timeout_id);
            break;
        }
        case EV_STATS_TIMEOUT:
        {
            transition(&FlightStats::state_ascending);
            break;
        }
        default:
        {
            break;
        }
    }
}
void FlightStats::state_ascending(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("[FlightStats] Entering ASCENDING state\n");

            state = State::ASCENDING;
            break;
        }
        case EV_EXIT:
        {
            TRACE("[FlightStats] Exiting ASCENDING state\n");

            LoggerProxy::getInstance()->log(apogee_stats);
            break;
        }
        case EV_APOGEE:
        {
            apogee_stats.T_apogee = static_cast<uint32_t>(miosix::getTick());
            transition(&FlightStats::state_drogueDeployment);
            break;
        }
        default:
        {
            break;
        }
    }
}

void FlightStats::state_drogueDeployment(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("[FlightStats] Entering DROGUE_DPL state\n");

            state = State::DROGUE_DPL;

            ev_timeout_id = sEventBroker->postDelayed(
                {EV_STATS_TIMEOUT}, TOPIC_STATS,
                FlightStatsConfig::TIMEOUT_STATE_DROGUE_DPL);
            break;
        }
        case EV_EXIT:
        {
            TRACE("[FlightStats] Entering EXITING state\n");

            LoggerProxy::getInstance()->log(drogue_dpl_stats);

            sEventBroker->removeDelayed(ev_timeout_id);
            break;
        }
        case EV_STATS_TIMEOUT:
        {
            transition(&FlightStats::state_idle);
            break;
        }
        case EV_DPL_ALTITUDE:
        {
            deferred_events.put(ev);
            break;
        }
        default:
        {
            break;
        }
    }
}

void FlightStats::state_mainDeployment(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("[FlightStats] Entering MAIN DPL state\n");

            state                = State::MAIN_DPL;
            main_dpl_stats.T_dpl = static_cast<uint32_t>(miosix::getTick());

            ev_timeout_id = sEventBroker->postDelayed(
                {EV_STATS_TIMEOUT}, TOPIC_STATS,
                FlightStatsConfig::TIMEOUT_STATE_MAIN_DPL);
            break;
        }
        case EV_EXIT:
        {
            TRACE("[FlightStats] Exiting MAIN DPL state\n");

            LoggerProxy::getInstance()->log(main_dpl_stats);

            sEventBroker->removeDelayed(ev_timeout_id);
            break;
        }
        case EV_STATS_TIMEOUT:
        {
            transition(&FlightStats::state_idle);
            break;
        }
        default:
        {
            break;
        }
    }
}
}  // namespace DeathStackBoard