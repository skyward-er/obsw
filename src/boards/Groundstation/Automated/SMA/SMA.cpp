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

#include "SMA.h"

#include <Groundstation/Automated/Actuators/Actuators.h>
#include <Groundstation/Automated/Config/SMAConfig.h>
#include <Groundstation/Automated/Hub.h>
#include <Groundstation/Automated/Leds/Leds.h>
#include <Groundstation/Automated/Sensors/Sensors.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <sensors/Vectornav/VN300/VN300Data.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#include "SMAData.h"

using namespace Boardcore;
using namespace Groundstation;
using namespace Common;
using namespace miosix;
using namespace std;

namespace Antennas
{

SMA::SMA(TaskScheduler* sched)
    : HSM(&SMA::state_config), scheduler(sched),
      propagator(SMAConfig::UPDATE_PERIOD), follower(SMAConfig::UPDATE_PERIOD)
{
    EventBroker::getInstance().subscribe(this, TOPIC_ARP);
    EventBroker::getInstance().subscribe(this, TOPIC_TMTC);
}

bool SMA::start()
{
    size_t result =
        scheduler->addTask(bind(&SMA::update, this), SMAConfig::UPDATE_PERIOD,
                           TaskScheduler::Policy::RECOVER);
    return HSM::start() && result != 0;
}

void SMA::setAntennaCoordinates(const Boardcore::GPSData& antennaCoordinates)
{
    if (!testState(&SMA::state_insert_info) &&
        !testState(&SMA::state_arm_ready) &&
        !testState(&SMA::state_fix_antennas))
    {
        LOG_ERR(logger,
                "Antenna coordinates can only be set in states: "
                "FIX_ANTENNAS, ARM_READY,  INSERT_INFO");
    }
    else
    {
        follower.setAntennaCoordinates(antennaCoordinates);
        EventBroker::getInstance().post(ARP_FIX_ANTENNAS, TOPIC_ARP);
    }
}

void SMA::setInitialRocketCoordinates(
    const Boardcore::GPSData& rocketCoordinates)
{
    if (!testState(&SMA::state_fix_rocket) &&
        !testState(&SMA::state_fix_rocket_nf))
    {
        LOG_ERR(logger,
                "Rocket coordinates can only be set in the "
                "FIX_ROCKET or FIX_ROCKET_NF state");
    }
    else
    {
        follower.setInitialRocketCoordinates(rocketCoordinates);
        EventBroker::getInstance().post(ARP_FIX_ROCKET, TOPIC_ARP);
    }
}

ErrorMovement SMA::moveStepperDeg(StepperList stepperId, float angle)
{
    if (!testState(&SMA::state_test) && !testState(&SMA::state_test_nf))
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

ErrorMovement SMA::moveStepperSteps(StepperList stepperId, int16_t steps)
{
    if (!testState(&SMA::state_test) && !testState(&SMA::state_test_nf))
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

void SMA::update()
{
    switch (status.state)
    {
        // in fix_antennas state, wait for the GPS fix of ARP
        case SMAState::FIX_ANTENNAS:
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
        case SMAState::FIX_ROCKET:
        case SMAState::FIX_ROCKET_NF:
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
        case SMAState::ACTIVE:
        case SMAState::ACTIVE_NF:
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
State SMA::state_config(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMAState::CONFIG);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMA::state_top);
        }
        case EV_INIT:
        {
            return transition(&SMA::state_init);
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
State SMA::state_feedback(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMAState::FEEDBACK);
            return HANDLED;
        }
        case EV_EXIT:
        {
            ModuleManager::getInstance().get<Actuators>()->disarm();
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMA::state_top);
        }
        case EV_INIT:
        {
            return transition(&SMA::state_armed);
        }
        case TMTC_ARP_DISARM:
        {
            return transition(&SMA::state_init_done);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

// Super state
State SMA::state_no_feedback(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMAState::NO_FEEDBACK);
            return HANDLED;
        }
        case EV_EXIT:
        {
            ModuleManager::getInstance().get<Actuators>()->disarm();
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMA::state_top);
        }
        case EV_INIT:
        {
            return transition(&SMA::state_armed_nf);
        }
        case TMTC_ARP_DISARM:
        {
            return transition(&SMA::state_arm_ready);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State SMA::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMAState::INIT);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMA::state_config);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case ARP_INIT_OK:
        {
            return transition(&SMA::state_init_done);
        }
        case ARP_INIT_ERROR:
        {
            return transition(&SMA::state_init_error);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State SMA::state_init_error(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMAState::INIT_ERROR);
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
            return tranSuper(&SMA::state_config);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_ARP_FORCE_NO_FEEDBACK:
        {
            return transition(&SMA::state_insert_info);
        }
        case TMTC_ARP_FORCE_INIT:
        {
            return transition(&SMA::state_init_done);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State SMA::state_init_done(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMAState::INIT_DONE);
            ModuleManager::getInstance().get<Leds>()->setOn(LedColor::GREEN);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMA::state_config);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_ARP_FORCE_NO_FEEDBACK:
        {
            return transition(&SMA::state_insert_info);
        }
        case TMTC_ARP_ARM:
        {
            return transition(&SMA::state_feedback);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State SMA::state_insert_info(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMAState::INSERT_INFO);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMA::state_config);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case ARP_FIX_ANTENNAS:
        {
            return transition(&SMA::state_arm_ready);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State SMA::state_arm_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMAState::ARM_READY);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMA::state_config);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_ARP_ARM:
        {
            return transition(&SMA::state_no_feedback);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State SMA::state_armed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMAState::ARMED);
            ModuleManager::getInstance().get<Actuators>()->arm();
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMA::state_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_ARP_FOLLOW:
        {
            return transition(&SMA::state_fix_antennas);
        }
        case TMTC_ARP_ENTER_TEST_MODE:
        {
            return transition(&SMA::state_test);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State SMA::state_test(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMAState::TEST);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMA::state_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_ARP_CALIBRATE:
        {
            return transition(&SMA::state_calibrate);
        }
        case TMTC_ARP_EXIT_TEST_MODE:
        {
            return transition(&SMA::state_armed);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State SMA::state_calibrate(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMAState::CALIBRATE);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMA::state_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case ARP_CAL_DONE:
        {
            return transition(&SMA::state_test);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State SMA::state_fix_antennas(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMAState::FIX_ANTENNAS);
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
            return tranSuper(&SMA::state_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case ARP_FIX_ANTENNAS:
        {
            return transition(&SMA::state_fix_rocket);
        }
        case TMTC_ARP_RESET_ALGORITHM:
        {
            return transition(&SMA::state_armed);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State SMA::state_fix_rocket(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMAState::FIX_ROCKET);
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
            return tranSuper(&SMA::state_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case ARP_FIX_ROCKET:
        {
            return transition(&SMA::state_active);
        }
        case TMTC_ARP_RESET_ALGORITHM:
        {
            return transition(&SMA::state_armed);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State SMA::state_active(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMAState::ACTIVE);
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
            return tranSuper(&SMA::state_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_ARP_RESET_ALGORITHM:
        {
            return transition(&SMA::state_armed);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State SMA::state_armed_nf(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMAState::ARMED_NF);
            ModuleManager::getInstance().get<Actuators>()->arm();
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMA::state_no_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_ARP_FOLLOW:
        {
            return transition(&SMA::state_fix_rocket_nf);
        }
        case TMTC_ARP_ENTER_TEST_MODE:
        {
            return transition(&SMA::state_test_nf);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State SMA::state_test_nf(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMAState::TEST_NF);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&SMA::state_no_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_ARP_EXIT_TEST_MODE:
        {
            return transition(&SMA::state_armed_nf);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State SMA::state_fix_rocket_nf(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMAState::FIX_ROCKET_NF);
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
            return tranSuper(&SMA::state_no_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case ARP_FIX_ROCKET:
        {
            return transition(&SMA::state_active_nf);
        }
        case TMTC_ARP_RESET_ALGORITHM:
        {
            return transition(&SMA::state_armed_nf);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State SMA::state_active_nf(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(SMAState::ACTIVE_NF);
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
            return tranSuper(&SMA::state_no_feedback);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_ARP_RESET_ALGORITHM:
        {
            return transition(&SMA::state_armed_nf);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

void SMA::logStatus(SMAState state)
{
    {
        PauseKernelLock lock;
        status.timestamp = TimestampTimer::getTimestamp();
        status.state     = state;
    }

    Logger::getInstance().log(status);
}

}  // namespace Antennas
