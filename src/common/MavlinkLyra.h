/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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
#include <mavlink_lib/lyra/mavlink.h>
#pragma GCC diagnostic pop

#warning "Lyra message definitions are deprecated, consider updating"

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
        case MAV_CMD_OPEN_NITROGEN:
            return TMTC_OPEN_NITROGEN;
        default:
            return LAST_EVENT;
    }
}

}  // namespace Common
