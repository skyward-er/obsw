/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

// Ignore warnings as these are auto-generated headers made by a third party
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <mavlink_lib/orion/mavlink.h>
#pragma GCC diagnostic pop

#include <common/Events.h>

namespace Common
{

inline Events mavCmdToEvent(uint8_t id)
{
    switch (id)
    {
        case MAV_CMD_ARM:
            return TMTC_ARM;
        case MAV_CMD_DISARM:
            return TMTC_DISARM;
        case MAV_CMD_CALIBRATE:
            return TMTC_CALIBRATE;
        case MAV_CMD_FORCE_INIT:
            return TMTC_FORCE_INIT;
        case MAV_CMD_FORCE_LAUNCH:
            return TMTC_FORCE_LAUNCH;
        case MAV_CMD_FORCE_ENGINE_SHUTDOWN:
            return TMTC_FORCE_ENGINE_SHUTDOWN;
        case MAV_CMD_FORCE_EXPULSION:
            return TMTC_FORCE_EXPULSION;
        case MAV_CMD_FORCE_DEPLOYMENT:
            return TMTC_FORCE_DEPLOYMENT;
        case MAV_CMD_FORCE_LANDING:
            return TMTC_FORCE_LANDING;
        case MAV_CMD_START_LOGGING:
            return TMTC_START_LOGGING;
        case MAV_CMD_STOP_LOGGING:
            return TMTC_STOP_LOGGING;
        case MAV_CMD_FORCE_REBOOT:
            return TMTC_RESET_BOARD;
        case MAV_CMD_ENTER_TEST_MODE:
            return TMTC_ENTER_TEST_MODE;
        case MAV_CMD_EXIT_TEST_MODE:
            return TMTC_EXIT_TEST_MODE;
        case MAV_CMD_ENTER_HIL:
            return TMTC_ENTER_HIL_MODE;
        case MAV_CMD_EXIT_HIL:
            return TMTC_EXIT_HIL_MODE;
        case MAV_CMD_START_RECORDING:
            return TMTC_START_RECORDING;
        case MAV_CMD_STOP_RECORDING:
            return TMTC_STOP_RECORDING;
        case MAV_CMD_OPEN_CHAMBER:
            return TMTC_OPEN_CHAMBER;
        default:
            return LAST_EVENT;
    }
}

inline const char* servoToString(ServosList servo)
{
    switch (servo)
    {
        case ServosList::AIR_BRAKES_SERVO:
            return "AIR_BRAKES_SERVO";
        case ServosList::EXPULSION_SERVO:
            return "EXPULSION_SERVO";
        case ServosList::PARAFOIL_LEFT_SERVO:
            return "PARAFOIL_LEFT_SERVO";
        case ServosList::PARAFOIL_RIGHT_SERVO:
            return "PARAFOIL_RIGHT_SERVO";
        case ServosList::OX_FILLING_VALVE:
            return "OX_FILLING_VALVE";
        case ServosList::OX_RELEASE_VALVE:
            return "OX_RELEASE_VALVE";
        case ServosList::OX_DETACH_SERVO:
            return "OX_DETACH_SERVO";
        case ServosList::OX_VENTING_VALVE:
            return "OX_VENTING_VALVE";
        case ServosList::N2_FILLING_VALVE:
            return "N2_FILLING_VALVE";
        case ServosList::N2_RELEASE_VALVE:
            return "N2_RELEASE_VALVE";
        case ServosList::N2_DETACH_SERVO:
            return "N2_DETACH_SERVO";
        case ServosList::N2_QUENCHING_VALVE:
            return "N2_QUENCHING_VALVE";
        case ServosList::N2_3WAY_VALVE:
            return "N2_3WAY_VALVE";
        case ServosList::MAIN_VALVE:
            return "MAIN_VALVE";
        case ServosList::NITROGEN_VALVE:
            return "NITROGEN_VALVE";
        default:
            return "UNKNOWN_SERVO";
    }
}

}  // namespace Common
