/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Federico Lolli, Nicolò Caruso
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
        antennaCoordinatesSet = true;
        EventBroker::getInstance().post(ARP_FIX_ANTENNAS, TOPIC_ARP);
    }
}

void SMA::setRocketNASOrigin(const Boardcore::GPSData& rocketCoordinates)
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
        follower.setRocketNASOrigin(rocketCoordinates);
        EventBroker::getInstance().post(ARP_FIX_ROCKET, TOPIC_ARP);
    }
}

ActuationStatus SMA::moveStepperDeg(StepperList stepperId, float angle)
{
    if (!testState(&SMA::state_test) && !testState(&SMA::state_test_nf))
    {
        LOG_ERR(logger, "Stepper can only be manually moved in the TEST state");
        return ActuationStatus::NOT_TEST;
    }
    else
    {
        return getModule<Actuators>()->moveDeg(stepperId, angle);
    }
}

ActuationStatus SMA::moveStepperSteps(StepperList stepperId, int16_t steps)
{
    if (!testState(&SMA::state_test) && !testState(&SMA::state_test_nf))
    {
        LOG_ERR(logger, "Stepper can only be manually moved in the TEST state");
        return ActuationStatus::NOT_TEST;
    }
    else
    {
        return getModule<Actuators>()->move(stepperId, steps);
    }
}

void SMA::setMultipliers(StepperList axis, float multiplier)
{
    if (!testState(&SMA::state_insert_info) && !testState(&SMA::state_test) &&
        !testState(&SMA::state_test_nf))
    {
        LOG_ERR(logger,
                "Stepper multipliers can only be set in the "
                "INSERT_INFO or TEST state");
    }
    else
    {
        getModule<Actuators>()->setMultipliers(axis, multiplier);
    }
}

void SMA::setFatal() { fatalInit = true; };

void SMA::update()
{
    GPSData rocketCoordinates, antennaCoordinates;
    VN300Data vn300Data;

    Hub* hub          = static_cast<Hub*>(getModule<Groundstation::HubBase>());
    auto* sensors     = getModule<Sensors>();
    rocketCoordinates = hub->getRocketOrigin();

    // Update the antenna position except in case of no feedback
    if (status.state != SMAState::FIX_ROCKET_NF &&
        status.state != SMAState::INIT_ERROR &&
        status.state != SMAState::ACTIVE_NF &&
        status.state != SMAState::ARM_READY &&
        status.state != SMAState::ACTIVE_NF)
    {
        // update antenna coordinates
        vn300Data = sensors->getVN300LastSample();
        if (vn300Data.fix_gps == 3)
        {
            // build the GPSData struct with the VN300 data
            antennaCoordinates.gpsTimestamp  = vn300Data.insTimestamp;
            antennaCoordinates.latitude      = vn300Data.latitude;
            antennaCoordinates.longitude     = vn300Data.longitude;
            antennaCoordinates.height        = vn300Data.altitude;
            antennaCoordinates.velocityNorth = vn300Data.nedVelX;
            antennaCoordinates.velocityEast  = vn300Data.nedVelY;
            antennaCoordinates.velocityDown  = vn300Data.nedVelZ;
            antennaCoordinates.satellites    = vn300Data.fix_gps;
            antennaCoordinates.fix           = vn300Data.fix_gps;

            // update follower with coordinates
            follower.setAntennaCoordinates(antennaCoordinates);
        }
    }

    // update follower with the rocket GPS data
    follower.setRocketNASOrigin(rocketCoordinates);

    switch (status.state)
    {
        // when in insert_info state, wait for antenna fix (manual insertion)
        // and multipliers set
        case SMAState::INSERT_INFO:
        {
            if (antennaCoordinatesSet)
            {
                EventBroker::getInstance().post(ARP_INFO_INSERTED, TOPIC_ARP);
            }

            break;
        }
        // in fix_antennas state, wait for the GPS fix of ARP
        case SMAState::FIX_ANTENNAS:
        {
            if (antennaCoordinates.fix == 3)
            {
                // fix found, now move to the next state
                EventBroker::getInstance().post(ARP_FIX_ANTENNAS, TOPIC_ARP);
            }
            break;
        }
        // in fix_rocket state, wait for the GPS fix of the rocket
        case SMAState::FIX_ROCKET:
        case SMAState::FIX_ROCKET_NF:
        {
            if (rocketCoordinates.fix == 3)
            {
                LOG_INFO(Logging::getLogger("automated_antennas"),
                         "Rocket NAS position with fix acquired [{}, {}] [deg]",
                         rocketCoordinates.latitude,
                         rocketCoordinates.longitude);

                // fix found, now move to the next state
                EventBroker::getInstance().post(ARP_FIX_ROCKET, TOPIC_ARP);
            }
            break;
        }
        // in active state, update the follower and propagator inner states
        case SMAState::ACTIVE:
        {
            // retrieve the last NAS Rocket state
            if (hub->hasNasSet())
            {
                NASState nasState = hub->getRocketNasState();

                // update the propagator with the NAS state
                // and retrieve the propagated state
                propagator.setRocketNasState(nasState);
            }
            propagator.update();  // step the propagator
            PropagatorState predicted = propagator.getState();

            // update the follower with the propagated state
            follower.setLastRocketNasState(predicted.getNasState());
            follower.setLastAntennaAttitude(vn300Data);
            follower.update();  // step the follower
            FollowerState follow = follower.getState();

            // Log the target angles and propagations info
            AntennaAngles target  = follower.getTargetAngles();
            target.nrPropagations = predicted.nPropagations;
            Boardcore::Logger::getInstance().log(
                static_cast<Boardcore::AntennaAngles>(target));

            // actuate the steppers
            auto steppers = getModule<Actuators>();
            steppers->setSpeed(StepperList::STEPPER_X, follow.horizontalSpeed);
            steppers->setSpeed(StepperList::STEPPER_Y, follow.verticalSpeed);

            ActuationStatus actuation =
                steppers->moveDeg(StepperList::STEPPER_X, follow.yaw);
            if (actuation != ActuationStatus::OK)
            {
                LOG_ERR(
                    logger,
                    "Step antenna - STEPPER_X could not move or reached move "
                    "limit. Error: ",
                    actuation, "\n");
            }

            actuation = steppers->moveDeg(StepperList::STEPPER_Y, follow.pitch);
            if (actuation != ActuationStatus::OK)
            {
                LOG_ERR(
                    logger,
                    "Step antenna - STEPPER_Y could not move or reached move "
                    "limit. Error: ",
                    actuation, "\n");
            }

            break;
        }
        case SMAState::ACTIVE_NF:
        {
            VN300Data fakeAttitudeData;

            // retrieve the last NAS Rocket state
            if (hub->hasNasSet())
            {
                NASState nasState = hub->getRocketNasState();

                // update the propagator with the NAS state
                // and retrieve the propagated state
                propagator.setRocketNasState(nasState);
            }
            propagator.update();  // step the propagator
            PropagatorState predicted = propagator.getState();

            auto steppers = getModule<Actuators>();

            // set the attitude as the current position of the steppers
            // FIXME this method of setting the attitude is too dirty
            // if the follower is updated something may break here
            fakeAttitudeData.pitch =
                steppers->getCurrentDegPosition(StepperList::STEPPER_Y);
            fakeAttitudeData.yaw =
                steppers->getCurrentDegPosition(StepperList::STEPPER_X);

            // update the follower with the propagated state
            follower.setLastRocketNasState(predicted.getNasState());
            follower.setLastAntennaAttitude(fakeAttitudeData);
            follower.update();  // step the follower
            FollowerState follow = follower.getState();

            // Log the target angles and propagations info
            AntennaAngles target  = follower.getTargetAngles();
            target.nrPropagations = predicted.nPropagations;
            Boardcore::Logger::getInstance().log(
                static_cast<Boardcore::AntennaAngles>(target));

            // actuate the steppers
            steppers->setSpeed(StepperList::STEPPER_X, follow.horizontalSpeed);
            steppers->setSpeed(StepperList::STEPPER_Y, follow.verticalSpeed);

            ActuationStatus actuation =
                steppers->moveDeg(StepperList::STEPPER_X, follow.yaw);
            if (actuation != ActuationStatus::OK)
            {
                LOG_ERR(
                    logger,
                    "Step antenna - STEPPER_X could not move or reached move "
                    "limit. Error: ",
                    actuation, "\n");
            }

            actuation = steppers->moveDeg(StepperList::STEPPER_Y, follow.pitch);
            if (actuation != ActuationStatus::OK)
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
            getModule<Leds>()->setOn(LedColor::YELLOW);
            return HANDLED;
        }
        case EV_EXIT:
        {
            getModule<Actuators>()->disarm();
            getModule<Leds>()->setOff(LedColor::YELLOW);
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
            getModule<Leds>()->setOn(LedColor::YELLOW);
            return HANDLED;
        }
        case EV_EXIT:
        {
            getModule<Actuators>()->disarm();
            getModule<Leds>()->setOff(LedColor::YELLOW);
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
            if (fatalInit)
                getModule<Leds>()->setFastBlink(LedColor::RED);
            else
                getModule<Leds>()->setOn(LedColor::RED);
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
            getModule<Leds>()->setOff(LedColor::RED);
            getModule<Leds>()->setOff(LedColor::BLUE);
            getModule<Leds>()->setSlowBlink(LedColor::YELLOW);
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
            getModule<Leds>()->setOff(LedColor::YELLOW);
            getModule<Leds>()->setOn(LedColor::RED);
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
        case ARP_INFO_INSERTED:
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
            getModule<Leds>()->setOff(LedColor::BLUE);
            getModule<Leds>()->setOn(LedColor::RED);
            getModule<Leds>()->setSlowBlink(LedColor::YELLOW);
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
            getModule<Actuators>()->arm();
            getModule<Leds>()->setOn(LedColor::YELLOW);
            getModule<Leds>()->setOff(LedColor::BLUE);
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
            getModule<Leds>()->setSlowBlink(LedColor::BLUE);
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
            getModule<Leds>()->setFastBlink(LedColor::BLUE);
            return HANDLED;
        }
        case EV_EXIT:
        {
            auto* leds = getModule<Leds>();
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
            getModule<Leds>()->setOn(LedColor::BLUE);
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
            getModule<Actuators>()->arm();
            getModule<Leds>()->setOff(LedColor::BLUE);
            getModule<Leds>()->setOn(LedColor::YELLOW);
            getModule<Leds>()->setOn(LedColor::RED);
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
            getModule<Leds>()->setFastBlink(LedColor::BLUE);
            return HANDLED;
        }
        case EV_EXIT:
        {
            auto* leds = getModule<Leds>();
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
            getModule<Leds>()->setOn(LedColor::BLUE);
            getModule<Leds>()->setOn(LedColor::RED);
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
