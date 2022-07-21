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

namespace Main
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
    FLIGHT_LIFTOFF_DETECTED,
    FLIGHT_NC_DETACHED,
    FMM_ALGOS_CAL_DONE,
    FMM_INIT_OK,
    FMM_INIT_ERROR,
    FMM_MISSION_TIMEOUT,
    FMM_SENSORS_CAL_DONE,
    FSR_STATS_TIMEOUT,
    NAS_CALIBRATE,
    NAS_READY,
    TMTC_RESET_BOARD,
    TMTC_FORCE_INIT,
    TMTC_ARM,
    TMTC_CAL_SENSORS,
    TMTC_CAL_ALGOS,
    TMTC_DISARM,
    TMTC_ENTER_TEST_MODE,
    TMTC_EXIT_TEST_MODE,
    TMTC_FORCE_LAUNCH,
    TMTC_FORCE_DROGUE,
    TMTC_FORCE_MAIN,
    TMTC_FORCE_LANDING,
};

}  // namespace Main
