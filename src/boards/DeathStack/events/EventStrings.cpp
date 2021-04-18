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

// Autogen date: 2021-04-17 23:09:11.522877

#include <map>
using std::map;

#include "Events.h"
#include "Topics.h"

string getEventString(uint8_t event)
{
    static const map<uint8_t, string> event_string_map{
        {EV_CALIBRATE, "EV_CALIBRATE"},
        {EV_CALIBRATE_SENSORS, "EV_CALIBRATE_SENSORS"},
        {EV_ARMED, "EV_ARMED"},
        {EV_TC_START_SENSOR_LOGGING, "EV_TC_START_SENSOR_LOGGING"},
        {EV_SM_READY, "EV_SM_READY"},
        {EV_LANDED, "EV_LANDED"},
        {EV_TC_STOP_SENSOR_LOGGING, "EV_TC_STOP_SENSOR_LOGGING"},
        {EV_RESET_SERVO, "EV_RESET_SERVO"},
        {EV_WIGGLE_SERVO, "EV_WIGGLE_SERVO"},
        {EV_NC_OPEN, "EV_NC_OPEN"},
        {EV_CUT_DROGUE, "EV_CUT_DROGUE"},
        {EV_TEST_CUT_PRIMARY, "EV_TEST_CUT_PRIMARY"},
        {EV_TEST_CUT_BACKUP, "EV_TEST_CUT_BACKUP"},
        {EV_NC_DETACHED, "EV_NC_DETACHED"},
        {EV_NC_OPEN_TIMEOUT, "EV_NC_OPEN_TIMEOUT"},
        {EV_CUTTING_TIMEOUT, "EV_CUTTING_TIMEOUT"},
        {EV_LIFTOFF, "EV_LIFTOFF"},
        {EV_TEST_ABK, "EV_TEST_ABK"},
        {EV_SHADOW_MODE_TIMEOUT, "EV_SHADOW_MODE_TIMEOUT"},
        {EV_DISABLE_ABK, "EV_DISABLE_ABK"},
        {EV_APOGEE, "EV_APOGEE"},
        {EV_TEST_TIMEOUT, "EV_TEST_TIMEOUT"},
        {EV_CALIBRATE_NAS, "EV_CALIBRATE_NAS"},
        {EV_NAS_READY, "EV_NAS_READY"},
        {EV_SEND_TEST_TM, "EV_SEND_TEST_TM"},
        {EV_SEND_HR_TM, "EV_SEND_HR_TM"},
        {EV_SEND_LR_TM, "EV_SEND_LR_TM"},
        {EV_DISARMED, "EV_DISARMED"},
        {EV_DPL_ALTITUDE, "EV_DPL_ALTITUDE"},
        {EV_STATS_TIMEOUT, "EV_STATS_TIMEOUT"},
        {EV_CALIBRATE_ADA, "EV_CALIBRATE_ADA"},
        {EV_ADA_READY, "EV_ADA_READY"},
        {EV_TIMEOUT_SHADOW_MODE, "EV_TIMEOUT_SHADOW_MODE"},
        {EV_ADA_APOGEE_DETECTED, "EV_ADA_APOGEE_DETECTED"},
        {EV_TIMEOUT_PRESS_STABILIZATION, "EV_TIMEOUT_PRESS_STABILIZATION"},
        {EV_ADA_DPL_ALT_DETECTED, "EV_ADA_DPL_ALT_DETECTED"},
    };
    auto it = event_string_map.find(event);
    return it == event_string_map.end() ? "EV_UNKNOWN" : it->second;
}

string getTopicString(uint8_t topic)
{
	static const map<uint8_t, string> topic_string_map{
        {TOPIC_FLIGHT_EVENTS, "TOPIC_FLIGHT_EVENTS"},
        {TOPIC_SM, "TOPIC_SM"},
        {TOPIC_TC, "TOPIC_TC"},
        {TOPIC_DPL, "TOPIC_DPL"},
        {TOPIC_ABK, "TOPIC_ABK"},
        {TOPIC_NAS, "TOPIC_NAS"},
        {TOPIC_TMTC, "TOPIC_TMTC"},
        {TOPIC_STATS, "TOPIC_STATS"},
        {TOPIC_ADA, "TOPIC_ADA"},
	};
	auto it = topic_string_map.find(topic);
	return it == topic_string_map.end() ? "TOPIC_UNKNOWN" : it->second;
}