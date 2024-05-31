/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Federico Lolli, Nicolò Caruso
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

#include <Groundstation/Automated/Actuators/Actuators.h>
#include <Groundstation/Automated/Config/SMControllerConfig.h>
#include <Groundstation/Automated/Hub.h>
#include <Groundstation/Automated/Leds/Leds.h>
#include <Groundstation/Automated/Sensors/Sensors.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <sensors/Vectornav/VN300/VN300Data.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#include "SMControllerData.h"

using namespace Boardcore;
using namespace Groundstation;
using namespace Common;
using namespace miosix;
using namespace std;

namespace Antennas
{

SMController::SMController(TaskScheduler* sched)
    : HSM(&SMController::state_config), scheduler(sched),
      propagator(SMControllerConfig::UPDATE_PERIOD),
      follower(SMControllerConfig::UPDATE_PERIOD)
{
    EventBroker::getInstance().subscribe(this, TOPIC_ARP);
    EventBroker::getInstance().subscribe(this, TOPIC_TMTC);
}

bool SMController::start()
{
    size_t result;
    bool ok = true;

    // add the update task
    result = scheduler->addTask(bind(&SMController::update, this),
                                SMControllerConfig::UPDATE_PERIOD,
                                TaskScheduler::Policy::RECOVER);
    ok &= result != 0;

    return ActiveObject::start() && ok;
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

ErrorMovement SMController::moveStepperDeg(StepperList stepperId, float angle)
{
    if (!testState(&SMController::state_test) &&
        !testState(&SMController::state_test_nf))
    {
        LOG_ERR(logger, "Stepper can only be manually moved in the TEST state");
        return ErrorMovement::NOT_TEST;
    }
    else
    {
        return ModuleManager::getInstance().get<Actuators>()->moveDeg(stepperId,
                                                                      angle);
    }
}

ErrorMovement SMController::moveStepperSteps(StepperList stepperId,
                                             int16_t steps)
{
    if (!testState(&SMController::state_test) &&
        !testState(&SMController::state_test_nf))
    {
        LOG_ERR(logger, "Stepper can only be manually moved in the TEST state");
        return ErrorMovement::NOT_TEST;
    }
    else
    {
        return ModuleManager::getInstance().get<Actuators>()->move(stepperId,
                                                                   steps);
    }
}

void SMController::update()
{
    switch (status.state)
    {
        // in fix_antennas state, wait for the GPS fix of ARP
        case SMControllerState::FIX_ANTENNAS:
        {
            VN300Data vn300Data;
            GPSData antennaPosition;

            auto* sensors = ModuleManager::getInstance().get<Sensors>();

            vn300Data = sensors->getVN300LastSample();
            if (vn300Data.fix_gps != 0)
            {
                // build the GPSData struct with the VN300 data
                antennaPosition.gpsTimestamp  = vn300Data.insTimestamp;
                antennaPosition.latitude      = vn300Data.latitude;
                antennaPosition.longitude     = vn300Data.longitude;
                antennaPosition.height        = vn300Data.altitude;
                antennaPosition.velocityNorth = vn300Data.nedVelX;
                antennaPosition.velocityEast  = vn300Data.nedVelY;
                antennaPosition.velocityDown  = vn300Data.nedVelZ;
                antennaPosition.satellites    = vn300Data.fix_gps;
                antennaPosition.fix           = (vn300Data.fix_gps > 0);

                // update follower with coordinates
                follower.setAntennaCoordinates(antennaPosition);

                LOG_INFO(Logging::getLogger("automated_antennas"),
                         "Antenna GPS position acquired !coord [{}, {}] [deg]",
                         antennaPosition.latitude, antennaPosition.longitude);

                // fix found, now move to the next state
                EventBroker::getInstance().post(ARP_FIX_ANTENNAS, TOPIC_ARP);
            }
            break;
        }
        // in fix_rocket state, wait for the GPS fix of the rocket
        case SMControllerState::FIX_ROCKET:
        case SMControllerState::FIX_ROCKET_NF:
        {
            GPSData rocketCoordinates;

            Hub* hub =
                static_cast<Hub*>(ModuleManager::getInstance().get<HubBase>());

            rocketCoordinates = hub->getRocketCoordinates();
            if (rocketCoordinates.fix != 0)
            {
                // update follower with the rocket GPS data
                follower.setInitialRocketCoordinates(rocketCoordinates);

                LOG_INFO(Logging::getLogger("automated_antennas"),
                         "Rocket GPS position acquired [{}, {}] [deg]",
                         rocketCoordinates.latitude,
                         rocketCoordinates.longitude);

                // fix found, now move to the next state
                EventBroker::getInstance().post(ARP_FIX_ROCKET, TOPIC_ARP);
            }
            break;
        }
        // in active state, update the follower and propagator inner states
        case SMControllerState::ACTIVE:
        case SMControllerState::ACTIVE_NF:
        {
            // retrieve the last NAS Rocket state
            Hub* hub =
                static_cast<Hub*>(ModuleManager::getInstance().get<HubBase>());
            NASState nasState = hub->getRocketNasState();

            // update the propagator with the NAS state
            // and retrieve the propagated state
            propagator.setRocketNasState(nasState);
            propagator.update();  // step the propagator
            PropagatorState predicted = propagator.getState();

            // update the follower with the propagated state
            follower.setLastRocketNasState(predicted.getNasState());
            VN300Data vn300Data = ModuleManager::getInstance()
                                      .get<Sensors>()
                                      ->getVN300LastSample();
            follower.setLastAntennaAttitude(vn300Data);
            follower.update();  // step the follower
            FollowerState follow = follower.getState();

            // actuate the steppers
            auto steppers = ModuleManager::getInstance().get<Actuators>();
            steppers->setSpeed(StepperList::STEPPER_X, follow.horizontalSpeed);
            steppers->setSpeed(StepperList::STEPPER_Y, follow.verticalSpeed);

            ErrorMovement actuation =
                steppers->moveDeg(StepperList::STEPPER_X, follow.yaw);
            if (actuation != ErrorMovement::OK)
            {
                LOG_ERR(
                    logger,
                    "Step antenna - STEPPER_X could not move or reached move "
                    "limit. Error: ",
                    actuation, "\n");
            }

            actuation = steppers->moveDeg(StepperList::STEPPER_Y, follow.pitch);
            if (actuation != ErrorMovement::OK)
            {
                LOG_ERR(
                    logger,
                    "Step antenna - STEPPER_Y could not move or reached move "
                    "limit. Error: ",
                    actuation, "\n");
            }

            break;
        }
        default:
        {
            break;
        }
    }
}

// Super state
State SMController::state_config(const Event& event)
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
State SMController::state_feedback(const Event& event)
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
            ModuleManager::getInstance().get<Actuators>()->disarm();
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
State SMController::state_no_feedback(const Event& event)
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
            ModuleManager::getInstance().get<Actuators>()->disarm();
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

State SMController::state_init(const Event& event)
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

State SMController::state_init_error(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::INIT_ERROR);
            ModuleManager::getInstance().get<Leds>()->setSlowBlink(
                LedColor::RED);
            return HANDLED;
        }
        case EV_EXIT:
        {
            ModuleManager::getInstance().get<Leds>()->setOff(LedColor::RED);
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

State SMController::state_init_done(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::INIT_DONE);
            ModuleManager::getInstance().get<Leds>()->setOn(LedColor::GREEN);
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

State SMController::state_insert_info(const Event& event)
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

State SMController::state_armed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::ARMED);
            ModuleManager::getInstance().get<Actuators>()->arm();
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

State SMController::state_test(const Event& event)
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

State SMController::state_calibrate(const Event& event)
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

State SMController::state_fix_antennas(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::FIX_ANTENNAS);
            ModuleManager::getInstance().get<Leds>()->setFastBlink(
                LedColor::ORANGE);
            return HANDLED;
        }
        case EV_EXIT:
        {
            auto* leds = ModuleManager::getInstance().get<Leds>();
            leds->setOn(LedColor::ORANGE);
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

State SMController::state_fix_rocket(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::FIX_ROCKET);
            ModuleManager::getInstance().get<Leds>()->setFastBlink(
                LedColor::YELLOW);
            return HANDLED;
        }
        case EV_EXIT:
        {
            auto* leds = ModuleManager::getInstance().get<Leds>();
            leds->setOff(LedColor::YELLOW);

            // init the follower before leaving the state
            // (compute initial arp-rocket distance and bearing)
            if (!follower.init())
            {
                LOG_ERR(logger, "Follower initialization failed");
            }

            leds->setOn(LedColor::YELLOW);
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

State SMController::state_active(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::ACTIVE);
            follower.begin();
            propagator.begin();
            return HANDLED;
        }
        case EV_EXIT:
        {
            follower.end();
            propagator.end();
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

State SMController::state_armed_nf(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::ARMED_NF);
            ModuleManager::getInstance().get<Actuators>()->arm();
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

State SMController::state_test_nf(const Event& event)
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

State SMController::state_fix_rocket_nf(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::FIX_ROCKET_NF);
            ModuleManager::getInstance().get<Leds>()->setFastBlink(
                LedColor::YELLOW);
            return HANDLED;
        }
        case EV_EXIT:
        {
            auto* leds = ModuleManager::getInstance().get<Leds>();
            leds->setOff(LedColor::YELLOW);

            // init the follower before leaving the state
            // (compute initial arp-rocket distance and bearing)
            if (!follower.init())
            {
                LOG_ERR(logger, "Follower initialization failed");
            }

            leds->setOn(LedColor::YELLOW);
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

State SMController::state_active_nf(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMControllerState::ACTIVE_NF);
            follower.begin();
            propagator.begin();
            return HANDLED;
        }
        case EV_EXIT:
        {
            follower.end();
            propagator.end();
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
