/* Copyright (c) 2018-2020 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alvise de' Faveri Tron
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

/*
 ******************************************************************************
 *                  THIS FILE IS AUTOGENERATED. DO NOT EDIT.                  *
 ******************************************************************************
 */

// Autogen date: 2021-05-05 14:56:16.829239

#include <map>
using std::map;

#include "Events.h"
#include "Topics.h"

string getEventString(uint8_t event)
{
    static const map<uint8_t, string> event_string_map{
        {EV_ADA_APOGEE_DETECTED, "EV_ADA_APOGEE_DETECTED"},
        {EV_ADA_DPL_ALT_DETECTED, "EV_ADA_DPL_ALT_DETECTED"},
        {EV_ADA_READY, "EV_ADA_READY"},
        {EV_APOGEE, "EV_APOGEE"},
        {EV_ARMED, "EV_ARMED"},
        {EV_CALIBRATE, "EV_CALIBRATE"},
        {EV_CALIBRATE_ADA, "EV_CALIBRATE_ADA"},
        {EV_CALIBRATE_NAS, "EV_CALIBRATE_NAS"},
        {EV_CALIBRATE_SENSORS, "EV_CALIBRATE_SENSORS"},
        {EV_CALIBRATION_OK, "EV_CALIBRATION_OK"},
        {EV_CUTTING_TIMEOUT, "EV_CUTTING_TIMEOUT"},
        {EV_CUT_DROGUE, "EV_CUT_DROGUE"},
        {EV_DISABLE_ABK, "EV_DISABLE_ABK"},
        {EV_DISARMED, "EV_DISARMED"},
        {EV_DPL_ALTITUDE, "EV_DPL_ALTITUDE"},
        {EV_INIT_ERROR, "EV_INIT_ERROR"},
        {EV_INIT_OK, "EV_INIT_OK"},
        {EV_LANDED, "EV_LANDED"},
        {EV_LIFTOFF, "EV_LIFTOFF"},
        {EV_NAS_READY, "EV_NAS_READY"},
        {EV_NC_DETACHED, "EV_NC_DETACHED"},
        {EV_NC_OPEN, "EV_NC_OPEN"},
        {EV_NC_OPEN_TIMEOUT, "EV_NC_OPEN_TIMEOUT"},
        {EV_RESET_SERVO, "EV_RESET_SERVO"},
        {EV_SEND_HR_TM, "EV_SEND_HR_TM"},
        {EV_SEND_LR_TM, "EV_SEND_LR_TM"},
        {EV_SEND_TEST_TM, "EV_SEND_TEST_TM"},
        {EV_SHADOW_MODE_TIMEOUT, "EV_SHADOW_MODE_TIMEOUT"},
        {EV_SM_READY, "EV_SM_READY"},
        {EV_STATS_TIMEOUT, "EV_STATS_TIMEOUT"},
        {EV_TC_ABK_DISABLE, "EV_TC_ABK_DISABLE"},
        {EV_TC_ABK_RESET_SERVO, "EV_TC_ABK_RESET_SERVO"},
        {EV_TC_ABK_WIGGLE_SERVO, "EV_TC_ABK_WIGGLE_SERVO"},
        {EV_TC_ARM, "EV_TC_ARM"},
        {EV_TC_CALIBRATE, "EV_TC_CALIBRATE"},
        {EV_TC_CALIBRATE_ADA, "EV_TC_CALIBRATE_ADA"},
        {EV_TC_CALIBRATE_NAS, "EV_TC_CALIBRATE_NAS"},
        {EV_TC_CALIBRATE_SENSORS, "EV_TC_CALIBRATE_SENSORS"},
        {EV_TC_CLOSE_LOG, "EV_TC_CLOSE_LOG"},
        {EV_TC_CUT_DROGUE, "EV_TC_CUT_DROGUE"},
        {EV_TC_DISARM, "EV_TC_DISARM"},
        {EV_TC_DPL_RESET_SERVO, "EV_TC_DPL_RESET_SERVO"},
        {EV_TC_DPL_WIGGLE_SERVO, "EV_TC_DPL_WIGGLE_SERVO"},
        {EV_TC_END_MISSION, "EV_TC_END_MISSION"},
        {EV_TC_FORCE_INIT, "EV_TC_FORCE_INIT"},
        {EV_TC_LAUNCH, "EV_TC_LAUNCH"},
        {EV_TC_NC_OPEN, "EV_TC_NC_OPEN"},
        {EV_TC_RESET_BOARD, "EV_TC_RESET_BOARD"},
        {EV_TC_START_SENSOR_LOGGING, "EV_TC_START_SENSOR_LOGGING"},
        {EV_TC_STOP_SENSOR_LOGGING, "EV_TC_STOP_SENSOR_LOGGING"},
        {EV_TC_TEST_ABK, "EV_TC_TEST_ABK"},
        {EV_TC_TEST_CUT_BACKUP, "EV_TC_TEST_CUT_BACKUP"},
        {EV_TC_TEST_CUT_PRIMARY, "EV_TC_TEST_CUT_PRIMARY"},
        {EV_TC_TEST_MODE, "EV_TC_TEST_MODE"},
        {EV_TEST_ABK, "EV_TEST_ABK"},
        {EV_TEST_CUT_BACKUP, "EV_TEST_CUT_BACKUP"},
        {EV_TEST_CUT_PRIMARY, "EV_TEST_CUT_PRIMARY"},
        {EV_TEST_MODE, "EV_TEST_MODE"},
        {EV_TEST_TIMEOUT, "EV_TEST_TIMEOUT"},
        {EV_TIMEOUT_END_MISSION, "EV_TIMEOUT_END_MISSION"},
        {EV_TIMEOUT_PRESS_STABILIZATION, "EV_TIMEOUT_PRESS_STABILIZATION"},
        {EV_UMBILICAL_DETACHED, "EV_UMBILICAL_DETACHED"},
        {EV_WIGGLE_SERVO, "EV_WIGGLE_SERVO"},
    };
    auto it = event_string_map.find(event);
    return it == event_string_map.end() ? "EV_UNKNOWN" : it->second;
}

string getTopicString(uint8_t topic)
{
	static const map<uint8_t, string> topic_string_map{
        {TOPIC_ABK, "TOPIC_ABK"},
        {TOPIC_ADA, "TOPIC_ADA"},
        {TOPIC_DPL, "TOPIC_DPL"},
        {TOPIC_FLIGHT_EVENTS, "TOPIC_FLIGHT_EVENTS"},
        {TOPIC_FMM, "TOPIC_FMM"},
        {TOPIC_NAS, "TOPIC_NAS"},
        {TOPIC_SM, "TOPIC_SM"},
        {TOPIC_STATS, "TOPIC_STATS"},
        {TOPIC_TC, "TOPIC_TC"},
        {TOPIC_TMTC, "TOPIC_TMTC"},
	};
	auto it = topic_string_map.find(topic);
	return it == topic_string_map.end() ? "TOPIC_UNKNOWN" : it->second;
}