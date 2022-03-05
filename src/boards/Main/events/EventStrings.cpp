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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <map>

#include "Events.h"
#include "Topics.h"

using std::map;

string getEventString(uint8_t event)
{
    static const map<uint8_t, string> event_string_map{
        {ABK_DISABLE, "ABK_DISABLE"},
        {ABK_OPEN, "ABK_OPEN"},
        {ABK_RESET, "ABK_RESET"},
        {ABK_SHADOW_MODE_TIMEOUT, "ABK_SHADOW_MODE_TIMEOUT"},
        {ABK_WIGGLE, ""},
        {DPL_CUT_DROGUE, "DPL_CUT_DROGUE"},
        {DPL_CUT_TIMEOUT, "DPL_CUT_TIMEOUT"},
        {DPL_OPEN, "DPL_OPEN"},
        {DPL_OPEN_NC, "DPL_OPEN_NC"},
        {DPL_OPEN_NC_TIMEOUT, "DPL_OPEN_NC_TIMEOUT"},
        {DPL_RESET, "DPL_RESET"},
        {DPL_WIGGLE, "DPL_WIGGLE"},
        {FLIGHT_APOGEE_DETECTED, "FLIGHT_APOGEE_DETECTED"},
        {FLIGHT_LIFTOFF_DETECTED, "FLIGHT_LIFTOFF_DETECTED"},
        {FLIGHT_NC_DETACHED, "FLIGHT_NC_DETACHED"},
    };
    auto it = event_string_map.find(event);
    return it == event_string_map.end() ? "EV_UNKNOWN" : it->second;
}

string getTopicString(uint8_t topic)
{
    static const map<uint8_t, string> topic_string_map{
        {TOPIC_ABK, "TOPIC_ABK"},
        {TOPIC_DPL, "TOPIC_DPL"},
        {TOPIC_FLIGHT, "TOPIC_FLIGHT"},
    };
    auto it = topic_string_map.find(topic);
    return it == topic_string_map.end() ? "TOPIC_UNKNOWN" : it->second;
}
