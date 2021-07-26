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

#include "FlightStatsRecorder.h"

#include <cmath>

#include "LoggerService/LoggerService.h"
#include "System/StackLogger.h"
#include "events/EventBroker.h"
#include "events/Events.h"

#include "configs/SensorManagerConfig.h"

namespace DeathStackBoard
{

FlightStatsRecorder::FlightStatsRecorder()
    : FSM(&FlightStatsRecorder::state_idle)
{
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_DPL);
    sEventBroker->subscribe(this, TOPIC_STATS);
}

FlightStatsRecorder::~FlightStatsRecorder() { sEventBroker->unsubscribe(this); }

void FlightStatsRecorder::update(const ADAKalmanState& t)
{
    switch (state)
    {
        case State::LIFTOFF:
        {
            apogee_stats.ada_min_pressure = t.x0;
            break;
        }
        case State::ASCENDING:
        {
            if (t.x0 < apogee_stats.ada_min_pressure)
            {
                apogee_stats.ada_min_pressure = t.x0;
            }
            break;
        }
        default:
            break;
    }
}

void FlightStatsRecorder::update(const CurrentSensorData& t)
{
    switch (state)
    {
        case State::TESTING_CUTTER:
        {
            if (t.channel_id == SensorConfigs::ADC_CS_CUTTER_PRIMARY)
            {
                ++cutters_stats.n_samples_1;
                cutters_stats.cutter_1_avg += t.current;
            }
            else if (t.channel_id == SensorConfigs::ADC_CS_CUTTER_BACKUP)
            {
                ++cutters_stats.n_samples_2;
                cutters_stats.cutter_2_avg += t.current;
            }
            break;
        }
        default:
            break;
    }
}

void FlightStatsRecorder::update(const ADAData& t)
{
    switch (state)
    {
        case State::LIFTOFF:
        {
            if (t.vert_speed > liftoff_stats.vert_speed_max)
            {
                liftoff_stats.vert_speed_max = t.vert_speed;
                liftoff_stats.T_max_speed =
                    static_cast<uint32_t>(miosix::getTick());
                liftoff_stats.altitude_max_speed = t.msl_altitude;
            }
            break;
        }
        case State::ASCENDING:
        {
            if (t.msl_altitude > apogee_stats.baro_max_altitude)
            {
                apogee_stats.baro_max_altitude = t.msl_altitude;
            }
            break;
        }
        case State::MAIN_DPL:
        {
            // Only set it one time
            if (main_dpl_stats.altitude_dpl == 0)
            {
                main_dpl_stats.altitude_dpl   = t.msl_altitude;
                main_dpl_stats.vert_speed_dpl = t.vert_speed;
            }
            break;
        }
        default:
            break;
    }
}

void FlightStatsRecorder::update(const MS5803Data& t)
{
    switch (state)
    {
        case State::ASCENDING:
        {
            if (t.press < apogee_stats.digital_min_pressure)
            {
                apogee_stats.digital_min_pressure = t.press;
            }
            break;
        }
        default:
            break;
    }
}

void FlightStatsRecorder::update(const MPXHZ6130AData& t)
{
    switch (state)
    {
        case State::ASCENDING:
        {
            if (t.press < apogee_stats.static_min_pressure)
            {
                apogee_stats.static_min_pressure = t.press;
            }
            break;
        }
        default:
            break;
    }
}

void FlightStatsRecorder::update(const SSCDRRN015PDAData& t)
{
    switch (state)
    {
        case State::ASCENDING:
        {
            // TODO : CONVERT DYNAMIC PRESSURE TO SPEED
            /*if (t.press > liftoff_stats.airspeed_pitot_max)
            {
                liftoff_stats.airspeed_pitot_max = t.press;
            }*/
            break;
        }
        default:
            break;
    }
}

void FlightStatsRecorder::update(const SSCDANN030PAAData& t)
{
    switch (state)
    {
        case State::ASCENDING:
        {
            if (t.press > drogue_dpl_stats.max_dpl_vane_pressure)
            {
                drogue_dpl_stats.max_dpl_vane_pressure = t.press;
            }
            break;
        }
        default:
            break;
    }
}

void FlightStatsRecorder::update(const BMX160Data& t)
{
    switch (state)
    {
        case State::LIFTOFF:
        {
            if (fabs(t.accel_x) > liftoff_stats.acc_max)
            {
                liftoff_stats.T_max_acc =
                    static_cast<uint32_t>(miosix::getTick());
                liftoff_stats.acc_max = fabs(t.accel_z);
            }
            break;
        }
        case State::DROGUE_DPL:
        {
            if (fabs(t.accel_z) > fabs(drogue_dpl_stats.max_dpl_acc))
            {
                drogue_dpl_stats.max_dpl_acc = t.accel_z;
                drogue_dpl_stats.T_dpl =
                    static_cast<uint32_t>(miosix::getTick());
            }
            break;
        }
        case State::MAIN_DPL:
        {
            if (fabs(t.accel_z) > fabs(main_dpl_stats.max_dpl_acc))
            {
                main_dpl_stats.max_dpl_acc = t.accel_z;
            }
            break;
        }
        default:
            break;
    }
}

void FlightStatsRecorder::update(const UbloxGPSData& t)
{
    switch (state)
    {
        case State::ASCENDING:
        {
            if (t.height > apogee_stats.gps_max_altitude)
            {
                apogee_stats.gps_max_altitude = t.height;
                apogee_stats.lat_apogee       = static_cast<float>(t.latitude);
                apogee_stats.lon_apogee       = static_cast<float>(t.longitude);
            }
            break;
        }
        default:
            break;
    }
}

void FlightStatsRecorder::state_idle(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("[FlightStats] Entering IDLE state\n");
            state = State::IDLE;

            StackLogger::getInstance()->updateStack(THID_STATS_FSM);
            break;
        }
        case EV_EXIT:
        {
            TRACE("[FlightStats] Exiting IDLE state\n");

            break;
        }
        case EV_LIFTOFF:
        {
            transition(&FlightStatsRecorder::state_liftOff);
            break;
        }
        case EV_TEST_CUT_BACKUP:
        case EV_TEST_CUT_PRIMARY:
        {
            transition(&FlightStatsRecorder::state_testingCutters);
            break;
        }
        case EV_DPL_ALTITUDE:
        {
            transition(&FlightStatsRecorder::state_mainDeployment);
            break;
        }
        default:
        {
            break;
        }
    }
}

void FlightStatsRecorder::state_testingCutters(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            cutters_stats = CutterTestStats{};

            state = State::TESTING_CUTTER;

            const int timeout = CutterConfig::CUT_TEST_DURATION;

            ev_timeout_id = sEventBroker->postDelayed<timeout>(
                {EV_STATS_TIMEOUT}, TOPIC_STATS);

            StackLogger::getInstance()->updateStack(THID_STATS_FSM);
            TRACE("[FlightStats] Entering CUTTER_TEST state\n");
            break;
        }
        case EV_EXIT:
        {
            cutters_stats.cutter_1_avg =
                cutters_stats.cutter_1_avg / cutters_stats.n_samples_1;
            cutters_stats.cutter_2_avg =
                cutters_stats.cutter_2_avg / cutters_stats.n_samples_2;

            LoggerService::getInstance()->log(cutters_stats);
            sEventBroker->removeDelayed(ev_timeout_id);

            TRACE("[FlightStats] Exiting CUTTER_TEST state\n");
            break;
        }
        case EV_STATS_TIMEOUT:
        {
            transition(&FlightStatsRecorder::state_idle);
            break;
        }
        default:
        {
            break;
        }
    }
}

void FlightStatsRecorder::state_liftOff(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("[FlightStats] Entering LIFTOFF state\n");
            state = State::LIFTOFF;

            // Collect liftoff stats until this event is received
            ev_timeout_id =
                sEventBroker
                    ->postDelayed<FlightStatsConfig::TIMEOUT_LIFTOFF_STATS>(
                        {EV_STATS_TIMEOUT}, TOPIC_STATS);

            // Save liftoff time
            liftoff_stats.T_liftoff = static_cast<uint32_t>(miosix::getTick());

            StackLogger::getInstance()->updateStack(THID_STATS_FSM);
            break;
        }
        case EV_EXIT:
        {
            TRACE("[FlightStats] Exiting LIFTOFF state\n");

            LoggerService::getInstance()->log(liftoff_stats);

            sEventBroker->removeDelayed(ev_timeout_id);
            break;
        }
        case EV_STATS_TIMEOUT:
        {
            transition(&FlightStatsRecorder::state_ascending);
            break;
        }
        default:
        {
            break;
        }
    }
}
void FlightStatsRecorder::state_ascending(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("[FlightStats] Entering ASCENDING state\n");

            state = State::ASCENDING;
            StackLogger::getInstance()->updateStack(THID_STATS_FSM);
            break;
        }
        case EV_EXIT:
        {
            TRACE("[FlightStats] Exiting ASCENDING state\n");

            LoggerService::getInstance()->log(apogee_stats);

            sEventBroker->removeDelayed(ev_timeout_id);
            break;
        }
        case EV_APOGEE:
        {
            // We reached apogee
            apogee_stats.T_apogee = static_cast<uint32_t>(miosix::getTick());

            // We detect apogee a little bit ahead of time, so wait a few
            // seconds in order to record the maximum altitude.
            ev_timeout_id =
                sEventBroker
                    ->postDelayed<FlightStatsConfig::TIMEOUT_APOGEE_STATS>(
                        {EV_STATS_TIMEOUT}, TOPIC_STATS);
            break;
        }
        case EV_STATS_TIMEOUT:
        {
            // Drogue deployment occurs just after apogee
            transition(&FlightStatsRecorder::state_drogueDeployment);
            break;
        }
        default:
        {
            break;
        }
    }
}

void FlightStatsRecorder::state_drogueDeployment(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("[FlightStats] Entering DROGUE_DPL state\n");

            state = State::DROGUE_DPL;

            // Collect stats until this event is received
            ev_timeout_id =
                sEventBroker
                    ->postDelayed<FlightStatsConfig::TIMEOUT_DROGUE_DPL_STATS>(
                        {EV_STATS_TIMEOUT}, TOPIC_STATS);

            StackLogger::getInstance()->updateStack(THID_STATS_FSM);
            break;
        }
        case EV_EXIT:
        {
            TRACE("[FlightStats] Entering EXITING state\n");

            LoggerService::getInstance()->log(drogue_dpl_stats);

            sEventBroker->removeDelayed(ev_timeout_id);
            break;
        }
        case EV_STATS_TIMEOUT:
        {
            transition(&FlightStatsRecorder::state_idle);
            break;
        }
        default:
        {
            break;
        }
    }
}

void FlightStatsRecorder::state_mainDeployment(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("[FlightStats] Entering MAIN DPL state\n");

            state = State::MAIN_DPL;

            // Save deployment timestamp
            main_dpl_stats.T_dpl = static_cast<uint32_t>(miosix::getTick());

            // Record stats until this event occurs
            ev_timeout_id =
                sEventBroker
                    ->postDelayed<FlightStatsConfig::TIMEOUT_MAIN_DPL_STATS>(
                        {EV_STATS_TIMEOUT}, TOPIC_STATS);

            StackLogger::getInstance()->updateStack(THID_STATS_FSM);
            break;
        }
        case EV_EXIT:
        {
            TRACE("[FlightStats] Exiting MAIN DPL state\n");

            LoggerService::getInstance()->log(main_dpl_stats);

            sEventBroker->removeDelayed(ev_timeout_id);
            break;
        }
        case EV_STATS_TIMEOUT:
        {
            transition(&FlightStatsRecorder::state_idle);
            break;
        }
        default:
        {
            break;
        }
    }
}
}  // namespace DeathStackBoard