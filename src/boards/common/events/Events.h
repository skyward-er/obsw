/* Copyright (c) 2018-2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#pragma once

#include <events/Event.h>
#include <events/EventBroker.h>

#include <cstdint>
#include <string>
#include <vector>

#include "Topics.h"

namespace Common
{

enum Events : uint8_t
{
    ABK_DISABLE = Boardcore::EV_FIRST_CUSTOM,
    ABK_OPEN,
    ABK_RESET,
    ABK_SHADOW_MODE_TIMEOUT,
    ABK_WIGGLE,
    ADA_CALIBRATE,
    ADA_PRESS_STAB_TIMEOUT,
    ADA_READY,
    ADA_SHADOW_MODE_TIMEOUT,
    DPL_CUT_DROGUE,
    DPL_CUT_TIMEOUT,
    DPL_OPEN,
    DPL_OPEN_NC,
    DPL_OPEN_NC_TIMEOUT,
    DPL_RESET,
    DPL_SERVO_ACTUATION_DETECTED,
    DPL_WIGGLE,
    FLIGHT_APOGEE_DETECTED,
    FLIGHT_ARMED,
    FLIGHT_DISARMED,
    FLIGHT_DPL_ALT_DETECTED,
    FLIGHT_LANDING_DETECTED,
    FLIGHT_UMBILICAL_DETACHED,
    FLIGHT_LIFTOFF,
    FLIGHT_MISSION_TIMEOUT,
    FLIGHT_NC_DETACHED,
    FLIGHT_WING_ALT_REACHED,
    FMM_ALGOS_CAL_DONE,
    FMM_INIT_OK,
    FMM_INIT_ERROR,
    FMM_SENSORS_CAL_DONE,
    FSR_STATS_TIMEOUT,
    NAS_CALIBRATE,
    NAS_READY,
    NAS_FORCE_START,
    NAS_FORCE_STOP,
    TMTC_ARM,
    TMTC_DISARM,
    TMTC_CALIBRATE,
    TMTC_FORCE_INIT,
    TMTC_FORCE_LAUNCH,
    TMTC_FORCE_LANDING,
    TMTC_FORCE_APOGEE,
    TMTC_FORCE_EXPULSION,
    TMTC_FORCE_MAIN,
    TMTC_START_LOGGING,
    TMTC_STOP_LOGGING,
    TMTC_RESET_BOARD,
    TMTC_ENTER_TEST_MODE,
    TMTC_EXIT_TEST_MODE,
    TMTC_START_RECORDING,
    TMTC_STOP_RECORDING,
};

inline string getEventString(uint8_t event)
{
    static const map<uint8_t, string> event_string_map{
        {ABK_DISABLE, "ABK_DISABLE"},
        {ABK_OPEN, "ABK_OPEN"},
        {ABK_RESET, "ABK_RESET"},
        {ABK_SHADOW_MODE_TIMEOUT, "ABK_SHADOW_MODE_TIMEOUT"},
        {ABK_WIGGLE, "ABK_WIGGLE"},
        {ADA_CALIBRATE, "ADA_CALIBRATE"},
        {ADA_PRESS_STAB_TIMEOUT, "ADA_PRESS_STAB_TIMEOUT"},
        {ADA_READY, "ADA_READY"},
        {ADA_SHADOW_MODE_TIMEOUT, "ADA_SHADOW_MODE_TIMEOUT"},
        {DPL_CUT_DROGUE, "DPL_CUT_DROGUE"},
        {DPL_CUT_TIMEOUT, "DPL_CUT_TIMEOUT"},
        {DPL_OPEN, "DPL_OPEN"},
        {DPL_OPEN_NC, "DPL_OPEN_NC"},
        {DPL_OPEN_NC_TIMEOUT, "DPL_OPEN_NC_TIMEOUT"},
        {DPL_RESET, "DPL_RESET"},
        {DPL_SERVO_ACTUATION_DETECTED, "DPL_SERVO_ACTUATION_DETECTED"},
        {DPL_WIGGLE, "DPL_WIGGLE"},
        {FLIGHT_APOGEE_DETECTED, "FLIGHT_APOGEE_DETECTED"},
        {FLIGHT_ARMED, "FLIGHT_ARMED"},
        {FLIGHT_DISARMED, "FLIGHT_DISARMED"},
        {FLIGHT_DPL_ALT_DETECTED, "FLIGHT_DPL_ALT_DETECTED"},
        {FLIGHT_LANDING_DETECTED, "FLIGHT_LANDING_DETECTED"},
        {FLIGHT_UMBILICAL_DETACHED, "FLIGHT_UMBILICAL_DETACHED"},
        {FLIGHT_LIFTOFF, "FLIGHT_LIFTOFF"},
        {FLIGHT_NC_DETACHED, "FLIGHT_NC_DETACHED"},
        {FLIGHT_MISSION_TIMEOUT, "FLIGHT_MISSION_TIMEOUT"},
        {FLIGHT_WING_ALT_REACHED, "FLIGHT_WING_ALT_REACHED"},
        {FMM_ALGOS_CAL_DONE, "FMM_ALGOS_CAL_DONE"},
        {FMM_INIT_OK, "FMM_INIT_OK"},
        {FMM_INIT_ERROR, "FMM_INIT_ERROR"},
        {FMM_SENSORS_CAL_DONE, "FMM_SENSORS_CAL_DONE"},
        {FSR_STATS_TIMEOUT, "FSR_STATS_TIMEOUT"},
        {NAS_CALIBRATE, "NAS_CALIBRATE"},
        {NAS_READY, "NAS_READY"},
        {TMTC_ARM, "TMTC_ARM"},
        {TMTC_DISARM, "TMTC_DISARM"},
        {TMTC_CALIBRATE, "TMTC_CALIBRATE"},
        {TMTC_FORCE_INIT, "TMTC_FORCE_INIT"},
        {TMTC_FORCE_LAUNCH, "TMTC_FORCE_LAUNCH"},
        {TMTC_FORCE_LANDING, "TMTC_FORCE_LANDING"},
        {TMTC_FORCE_APOGEE, "TMTC_FORCE_APOGEE"},
        {TMTC_FORCE_EXPULSION, "TMTC_FORCE_EXPULSION"},
        {TMTC_FORCE_MAIN, "TMTC_FORCE_MAIN"},
        {TMTC_RESET_BOARD, "TMTC_RESET_BOARD"},
        {TMTC_ENTER_TEST_MODE, "TMTC_ENTER_TEST_MODE"},
        {TMTC_EXIT_TEST_MODE, "TMTC_EXIT_TEST_MODE"},
        {TMTC_START_RECORDING, "TMTC_START_RECORDING"},
        {TMTC_STOP_RECORDING, "TMTC_STOP_RECORDING"},
    };

    auto it = event_string_map.find(event);
    return it == event_string_map.end() ? "EV_UNKNOWN" : it->second;
}

}  // namespace Common
