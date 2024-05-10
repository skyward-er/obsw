/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Federico Lolli
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

#include "SMController.h"

#include <Groundstation/Automated/Config/PropagatorConfig.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>

#include "SMControllerData.h"

using namespace Boardcore;
using namespace Groundstation;
using namespace Common;
using namespace miosix;

namespace Antennas
{

SMController::SMController(TaskScheduler* sched)
    : HSM(&SMController::state_config), scheduler(sched),
      propagator(PropagatorConfig::PROPAGATOR_PERIOD), follower()
{
    EventBroker::getInstance().subscribe(this, TOPIC_ARP);
    EventBroker::getInstance().subscribe(this, TOPIC_TMTC);
}

void SMController::setAntennaCoordinates(
    const Boardcore::GPSData& antennaCoordinates)
{
    if (!testState(&SMController::state_insert_info) &&
        !testState(&SMController::state_fix_antennas))
    {
        LOG_ERR(logger,
                "Antenna coordinates can only be set in states: "
                "FIX_ANTENNAS, INSERT_INFO");
    }
    else
    {
        follower.setAntennaCoordinates(antennaCoordinates);
    }
}

void SMController::setInitialRocketCoordinates(
    const Boardcore::GPSData& rocketCoordinates)
{
    if (!testState(&SMController::state_fix_rocket) &&
        !testState(&SMController::state_fix_rocket_nf))
    {
        LOG_ERR(logger,
                "Rocket coordinates can only be set in the "
                "FIX_ROCKET or FIX_ROCKET_NF state");
    }
    else
    {
        follower.setInitialRocketCoordinates(rocketCoordinates);
    }
}

// Super state
Boardcore::State SMController::state_config(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::CONFIG);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMController::state_top);
        }
        case EV_INIT:
        {
            return transition(&SMController::state_init);
        }
        case TMTC_ARP_RESET_BOARD:
        {
            reboot();
            return HANDLED;
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

// Super state
Boardcore::State SMController::state_feedback(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::FEEDBACK);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMController::state_top);
        }
        case EV_INIT:
        {
            return transition(&SMController::state_armed);
        }
        case TMTC_ARP_DISARM:
        {
            return transition(&SMController::state_init_done);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

// Super state
Boardcore::State SMController::state_no_feedback(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::NO_FEEDBACK);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMController::state_top);
        }
        case EV_INIT:
        {
            return transition(&SMController::state_armed_nf);
        }
        case TMTC_ARP_DISARM:
        {
            return transition(&SMController::state_insert_info);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State SMController::state_init(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::INIT);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMController::state_config);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case ARP_INIT_OK:
        {
            return transition(&SMController::state_init_done);
        }
        case ARP_INIT_ERROR:
        {
            return transition(&SMController::state_init_error);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State SMController::state_init_error(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::INIT_ERROR);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMController::state_config);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_ARP_FORCE_NO_FEEDBACK:
        {
            return transition(&SMController::state_insert_info);
        }
        case TMTC_ARP_FORCE_INIT:
        {
            return transition(&SMController::state_init_done);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State SMController::state_init_done(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::INIT_DONE);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMController::state_config);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_ARP_FORCE_NO_FEEDBACK:
        {
            return transition(&SMController::state_insert_info);
        }
        case TMTC_ARP_ARM:
        {
            return transition(&SMController::state_feedback);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State SMController::state_insert_info(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::INSERT_INFO);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMController::state_config);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_ARP_ARM:
        {
            return transition(&SMController::state_no_feedback);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State SMController::state_armed(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::ARMED);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMController::state_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_ARP_ENTER_TEST_MODE:
        {
            return transition(&SMController::state_test);
        }
        case TMTC_ARP_CALIBRATE:
        {
            return transition(&SMController::state_calibrate);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State SMController::state_test(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::TEST);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMController::state_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_ARP_EXIT_TEST_MODE:
        {
            return transition(&SMController::state_armed);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State SMController::state_calibrate(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::CALIBRATE);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMController::state_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case ARP_CAL_DONE:
        {
            return transition(&SMController::state_fix_antennas);
        }
        case TMTC_ARP_RESET_ALGORITHM:
        {
            return transition(&SMController::state_armed);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State SMController::state_fix_antennas(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::FIX_ANTENNAS);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMController::state_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case ARP_FIX_ANTENNAS:
        {
            return transition(&SMController::state_fix_rocket);
        }
        case TMTC_ARP_RESET_ALGORITHM:
        {
            return transition(&SMController::state_armed);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State SMController::state_fix_rocket(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::FIX_ROCKET);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMController::state_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case ARP_FIX_ROCKET:
        {
            return transition(&SMController::state_active);
        }
        case TMTC_ARP_RESET_ALGORITHM:
        {
            return transition(&SMController::state_armed);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State SMController::state_active(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::ACTIVE);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMController::state_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_ARP_RESET_ALGORITHM:
        {
            return transition(&SMController::state_armed);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State SMController::state_armed_nf(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::ARMED_NF);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMController::state_no_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_ARP_ENTER_TEST_MODE:
        {
            return transition(&SMController::state_test_nf);
        }
        case TMTC_ARP_CALIBRATE:
        {
            return transition(&SMController::state_fix_rocket_nf);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State SMController::state_test_nf(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::TEST_NF);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMController::state_no_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_ARP_EXIT_TEST_MODE:
        {
            return transition(&SMController::state_armed_nf);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State SMController::state_fix_rocket_nf(
    const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::FIX_ROCKET_NF);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMController::state_no_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case ARP_FIX_ROCKET:
        {
            return transition(&SMController::state_active_nf);
        }
        case TMTC_ARP_RESET_ALGORITHM:
        {
            return transition(&SMController::state_armed_nf);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State SMController::state_active_nf(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::ACTIVE_NF);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMController::state_no_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_ARP_RESET_ALGORITHM:
        {
            return transition(&SMController::state_armed_nf);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

void SMController::logStatus(SMControllerState state)
{
    {
        PauseKernelLock lock;
        status.timestamp = TimestampTimer::getTimestamp();
        status.state     = state;
    }

    Logger::getInstance().log(status);
}

}  // namespace Antennas
