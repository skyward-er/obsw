/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Pietro Bortolus
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

#include "FiringSequenceHSM.h"

#include <Motor/BoardScheduler.h>
#include <Motor/Configs/FiringSequenceConfig.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

using namespace Boardcore;
using namespace Common;
using namespace miosix;
using namespace std::chrono;

namespace Motor
{

FiringSequenceHSM::FiringSequenceHSM()
    : HSM(&FiringSequenceHSM::state_ready, STACK_DEFAULT_FOR_PTHREAD,
          BoardScheduler::firingSequenceHSMPriority())
{
    EventBroker::getInstance().subscribe(this, TOPIC_FIRING_SEQUENCE);
}

void FiringSequenceHSM::setFiringParams(uint32_t fullThrottleTime,
                                        uint32_t lowThrottleTime,
                                        float oxPilotFlamePosition,
                                        float fuelPilotFlamePosition,
                                        float oxLowThrottlePosition,
                                        float fuelLowThrottlePosition)
{
    getModule<Registry>()->setUnsafe(CONFIG_ID_FULL_THROTTLE_TIME,
                                     fullThrottleTime);
    getModule<Registry>()->setUnsafe(CONFIG_ID_LOW_THROTTLE_TIME,
                                     lowThrottleTime);
    getModule<Registry>()->setUnsafe(CONFIG_ID_PILOT_FLAME_OX_POSITION,
                                     oxPilotFlamePosition);
    getModule<Registry>()->setUnsafe(CONFIG_ID_PILOT_FLAME_FUEL_POSITION,
                                     fuelPilotFlamePosition);
    getModule<Registry>()->setUnsafe(CONFIG_ID_LOW_THROTTLE_OX_POSITION,
                                     oxLowThrottlePosition);
    getModule<Registry>()->setUnsafe(CONFIG_ID_LOW_THROTTLE_FUEL_POSITION,
                                     fuelLowThrottlePosition);

    paramsSet = true;
}

void FiringSequenceHSM::setPressureThresholds(float igniterPressureThreshold,
                                              float pilotFlamePressureThreshold)
{
    getModule<Registry>()->setUnsafe(CONFIG_ID_IGNITER_PRESSURE_THRESHOLD,
                                     igniterPressureThreshold);
    getModule<Registry>()->setUnsafe(CONFIG_ID_PILOT_FLAME_PRESSURE_THRESHOLD,
                                     pilotFlamePressureThreshold);

    igniterPressureThreshold = getModule<Registry>()->getOrSetDefaultUnsafe(
        CONFIG_ID_IGNITER_PRESSURE_THRESHOLD,
        Config::FiringSequence::IGNITER_PRESSURE_THRESHOLD);

    pilotFlamePressureThreshold = getModule<Registry>()->getOrSetDefaultUnsafe(
        CONFIG_ID_PILOT_FLAME_PRESSURE_THRESHOLD,
        Config::FiringSequence::PILOT_FLAME_PRESSURE_THRESHOLD);
}

bool FiringSequenceHSM::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->firingSequenceHSM();

    igniterPressureThreshold = getModule<Registry>()->getOrSetDefaultUnsafe(
        CONFIG_ID_IGNITER_PRESSURE_THRESHOLD,
        Config::FiringSequence::IGNITER_PRESSURE_THRESHOLD);

    pilotFlamePressureThreshold = getModule<Registry>()->getOrSetDefaultUnsafe(
        CONFIG_ID_PILOT_FLAME_PRESSURE_THRESHOLD,
        Config::FiringSequence::PILOT_FLAME_PRESSURE_THRESHOLD);

    uint8_t igniterResult =
        scheduler.addTask([this]() { checkIgniterPressure(); },
                          Config::FiringSequence::UPDATE_RATE);

    uint8_t pilotFlameResult =
        scheduler.addTask([this]() { checkPilotFlamePressure(); },
                          Config::FiringSequence::UPDATE_RATE);

    if (igniterResult == 0 || pilotFlameResult == 0)
    {
        LOG_ERR(logger, "Failed to add firing sequence tasks");
        return false;
    }

    if (!HSM::start())
    {
        LOG_ERR(logger, "Failed to activate FiringSequence HSM thread");
        return false;
    }

    return true;
}

FiringSequenceState FiringSequenceHSM::getState() { return state; }

void FiringSequenceHSM::checkIgniterPressure()
{
    if (state == FiringSequenceState::IGNITER_WAIT)
    {
        PressureData igniterChamberPressure =
            getModule<Sensors>()->getIgniterChamberPressure();

        if (igniterChamberPressure.pressure > igniterPressureThreshold)
        {
            igniterFlameSamples++;

            if (igniterFlameSamples >=
                Config::FiringSequence::IGNITER_CONFIRMATION_SAMPLES)
            {
                printf("Igniter flame confirmed\n");
                Thread::sleep(10);
                EventBroker::getInstance().post(FIRING_SEQUENCE_IGNITER_OK,
                                                TOPIC_FIRING_SEQUENCE);
            }
        }
        else
        {
            // Reset samples if pressure is not above threshold
            igniterFlameSamples = 0;
        }
    }
}

void FiringSequenceHSM::checkPilotFlamePressure()
{
    PressureData chamberPressure = getModule<Sensors>()->getMainCCPressure();

    if (chamberPressure.pressure >=
        Config::FiringSequence::MAIN_CHAMBER_SAFETY_THRESHOLD)
    {
        // If the pressure exceeds the safety threshold, abort
        EventBroker::getInstance().post(FIRING_SEQUENCE_ABORT,
                                        TOPIC_FIRING_SEQUENCE);
        return;
    }

    if (state == FiringSequenceState::PILOT_FLAME_WAIT)
    {
        if (chamberPressure.pressure > pilotFlamePressureThreshold)
        {
            pilotFlameSamples++;

            if (pilotFlameSamples >=
                Config::FiringSequence::PILOT_FLAME_CONFIRMATION_SAMPLES)
            {
                EventBroker::getInstance().post(FIRING_SEQUENCE_PILOT_FLAME_OK,
                                                TOPIC_FIRING_SEQUENCE);
            }
        }
        else
        {
            // Reset samples if pressure is not above threshold
            pilotFlameSamples = 0;
        }
    }
}

State FiringSequenceHSM::state_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FiringSequenceState::READY);
            return HANDLED;
        }

        case EV_INIT:
        {
            getModule<Actuators>()->closeAllValves();
            return HANDLED;
        }

        case FIRING_SEQUENCE_START:
        {
            /* if (!paramsSet)
            {
                LOG_ERR(
                    logger,
                    "Firing parameters not set, cannot start firing sequence");
                return HANDLED;
            } */
            return transition(&FiringSequenceHSM::state_igniter);
        }

        case EV_EMPTY:
        {
            return tranSuper(&FiringSequenceHSM::state_top);
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FiringSequenceHSM::state_igniter(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FiringSequenceState::IGNITER);
            return HANDLED;
        }

        case EV_INIT:
        {
            // Open oxidizer solenoid
            getModule<Actuators>()->openValveWithTime(
                ServosList::IGNITION_OX_VALVE,
                Config::FiringSequence::IGN_OX_OPENING_TIME.count());

            // wait before opening fuel solenoid and starting spark plug
            nextEventId = EventBroker::getInstance().postDelayed(
                FIRING_SEQUENCE_IGN_FUEL, TOPIC_FIRING_SEQUENCE,
                Config::FiringSequence::IGN_FUEL_DELAY.count());
            return HANDLED;
        }

        case FIRING_SEQUENCE_IGN_FUEL:
        {
            // Open fuel solenoid
            getModule<Actuators>()->openValveWithTime(
                ServosList::IGNITION_FUEL_VALVE,
                Config::FiringSequence::IGN_FUEL_OPENING_TIME.count());

            // Start spark plug
            getModule<Actuators>()->startSparkPlugWithTime(
                Config::FiringSequence::SPARK_TIME.count());

            return transition(&FiringSequenceHSM::state_igniter_wait);
        }

        case FIRING_SEQUENCE_ABORT:
        {
            EventBroker::getInstance().removeDelayed(nextEventId);
            return transition(&FiringSequenceHSM::state_ready);
        }

        case EV_EMPTY:
        {
            return tranSuper(&FiringSequenceHSM::state_ready);
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FiringSequenceHSM::state_igniter_wait(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FiringSequenceState::IGNITER_WAIT);
            return HANDLED;
        }

        case EV_INIT:
        {
            // Reset igniter confirmation samples
            igniterFlameSamples = 0;
            return HANDLED;
        }

        case FIRING_SEQUENCE_IGNITER_OK:
        {
            return transition(&FiringSequenceHSM::state_pilot_flame);
        }

        case FIRING_SEQUENCE_ABORT:
        {
            return transition(&FiringSequenceHSM::state_ready);
        }

        case EV_EMPTY:
        {
            return tranSuper(&FiringSequenceHSM::state_igniter);
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FiringSequenceHSM::state_pilot_flame(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FiringSequenceState::PILOT_FLAME);
            return HANDLED;
        }

        case EV_INIT:
        {
            // Get or set valve position from registry
            float oxPilotPosition =
                getModule<Registry>()->getOrSetDefaultUnsafe(
                    CONFIG_ID_PILOT_FLAME_OX_POSITION,
                    Config::FiringSequence::PILOT_OX_POSITION);

            float fuelPilotPosition =
                getModule<Registry>()->getOrSetDefaultUnsafe(
                    CONFIG_ID_PILOT_FLAME_FUEL_POSITION,
                    Config::FiringSequence::PILOT_FUEL_POSITION);

            // open valves to pilot flame position
            getModule<Actuators>()->moveValve(ServosList::MAIN_OX_VALVE,
                                              oxPilotPosition);

            getModule<Actuators>()->moveValve(ServosList::MAIN_FUEL_VALVE,
                                              fuelPilotPosition);
            return transition(&FiringSequenceHSM::state_pilot_flame_wait);
        }

        case FIRING_SEQUENCE_ABORT:
        {
            return transition(&FiringSequenceHSM::state_ready);
        }

        case EV_EMPTY:
        {
            return tranSuper(&FiringSequenceHSM::state_igniter_wait);
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FiringSequenceHSM::state_pilot_flame_wait(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FiringSequenceState::PILOT_FLAME_WAIT);
            return HANDLED;
        }

        case EV_INIT:
        {
            nextEventId = EventBroker::getInstance().postDelayed(
                FIRING_SEQUENCE_PILOT_FLAME_TIMEOUT, TOPIC_FIRING_SEQUENCE,
                Config::FiringSequence::PILOT_FLAME_MAX_TIME.count());

            // Reset pilot flame confirmation samples
            pilotFlameSamples = 0;
            return HANDLED;
        }

        case FIRING_SEQUENCE_PILOT_FLAME_OK:
        {
            EventBroker::getInstance().removeDelayed(nextEventId);
            return transition(&FiringSequenceHSM::state_ramp_up);
        }

        case FIRING_SEQUENCE_PILOT_FLAME_TIMEOUT:
        {
            // Pilot flame has not been detected, close valves
            getModule<Actuators>()->closeValve(ServosList::MAIN_OX_VALVE);
            getModule<Actuators>()->closeValve(ServosList::MAIN_FUEL_VALVE);
            return HANDLED;
        }

        case FIRING_SEQUENCE_ABORT:
        {
            EventBroker::getInstance().removeDelayed(nextEventId);
            return transition(&FiringSequenceHSM::state_ready);
        }

        case EV_EMPTY:
        {
            return tranSuper(&FiringSequenceHSM::state_pilot_flame);
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FiringSequenceHSM::state_ramp_up(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FiringSequenceState::RAMP_UP);
            return HANDLED;
        }

        case EV_INIT:
        {
            // Animate both main valves to fully open over the ramp up time
            getModule<Actuators>()->animateValve(
                ServosList::MAIN_OX_VALVE, 1.0f,
                Config::FiringSequence::RAMP_UP_OPENING_TIME.count());
            getModule<Actuators>()->animateValve(
                ServosList::MAIN_FUEL_VALVE, 1.0f,
                Config::FiringSequence::RAMP_UP_OPENING_TIME.count());

            nextEventId = EventBroker::getInstance().postDelayed(
                FIRING_SEQUENCE_FULL_THROTTLE, TOPIC_FIRING_SEQUENCE,
                Config::FiringSequence::RAMP_UP_OPENING_TIME.count());

            return HANDLED;
        }

        case FIRING_SEQUENCE_ABORT:
        {
            EventBroker::getInstance().removeDelayed(nextEventId);
            return transition(&FiringSequenceHSM::state_ready);
        }

        case FIRING_SEQUENCE_FULL_THROTTLE:
        {
            return transition(&FiringSequenceHSM::state_full_throttle);
        }

        case EV_EMPTY:
        {
            return tranSuper(&FiringSequenceHSM::state_pilot_flame_wait);
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FiringSequenceHSM::state_full_throttle(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FiringSequenceState::FULL_THROTTLE);
            return HANDLED;
        }

        case EV_INIT:
        {
            // Both valves are fully open, wait to transition to low throttle
            uint32_t fullThrottleTime =
                getModule<Registry>()->getOrSetDefaultUnsafe(
                    CONFIG_ID_FULL_THROTTLE_TIME,
                    static_cast<uint32_t>(
                        Config::FiringSequence::FULL_THROTTLE_TIME.count()));

            nextEventId = EventBroker::getInstance().postDelayed(
                FIRING_SEQUENCE_LOW_THROTTLE, TOPIC_FIRING_SEQUENCE,
                fullThrottleTime);

            return HANDLED;
        }

        case FIRING_SEQUENCE_ABORT:
        {
            EventBroker::getInstance().removeDelayed(nextEventId);
            return transition(&FiringSequenceHSM::state_ready);
        }

        case FIRING_SEQUENCE_LOW_THROTTLE:
        {
            return transition(&FiringSequenceHSM::state_low_throttle);
        }

        case EV_EMPTY:
        {
            return tranSuper(&FiringSequenceHSM::state_ramp_up);
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FiringSequenceHSM::state_low_throttle(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FiringSequenceState::LOW_THROTTLE);
            return HANDLED;
        }

        case EV_INIT:
        {
            // retreive params from registry
            uint32_t lowThrottleTime =
                getModule<Registry>()->getOrSetDefaultUnsafe(
                    CONFIG_ID_LOW_THROTTLE_TIME,
                    static_cast<uint32_t>(
                        Config::FiringSequence::LOW_THROTTLE_TIME.count()));

            float lowThrottleOxPosition =
                getModule<Registry>()->getOrSetDefaultUnsafe(
                    CONFIG_ID_LOW_THROTTLE_OX_POSITION,
                    Config::FiringSequence::LOW_THROTTLE_OX_POSITION);

            float lowThrottleFuelPosition =
                getModule<Registry>()->getOrSetDefaultUnsafe(
                    CONFIG_ID_LOW_THROTTLE_FUEL_POSITION,
                    Config::FiringSequence::LOW_THROTTLE_FUEL_POSITION);

            // move valves to low throttle position
            getModule<Actuators>()->moveValve(ServosList::MAIN_OX_VALVE,
                                              lowThrottleOxPosition);

            getModule<Actuators>()->moveValve(ServosList::MAIN_FUEL_VALVE,
                                              lowThrottleFuelPosition);

            nextEventId = EventBroker::getInstance().postDelayed(
                FIRING_SEQUENCE_END, TOPIC_FIRING_SEQUENCE, lowThrottleTime);

            return HANDLED;
        }

        case FIRING_SEQUENCE_ABORT:
        {
            EventBroker::getInstance().removeDelayed(nextEventId);
            return transition(&FiringSequenceHSM::state_ready);
        }

        case FIRING_SEQUENCE_END:
        {
            return transition(&FiringSequenceHSM::state_ended);
        }

        case EV_EMPTY:
        {
            return tranSuper(&FiringSequenceHSM::state_full_throttle);
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FiringSequenceHSM::state_ended(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FiringSequenceState::ENDED);
            return HANDLED;
        }

        case EV_INIT:
        {
            EventBroker::getInstance().post(EREG_CLOSE, TOPIC_EREG_OX);
            EventBroker::getInstance().post(EREG_CLOSE, TOPIC_EREG_FUEL);
            getModule<Actuators>()->closeAllValves();
            return HANDLED;
        }

        case FIRING_SEQUENCE_ABORT:
        {
            return transition(&FiringSequenceHSM::state_ready);
        }

        case EV_EMPTY:
        {
            return tranSuper(&FiringSequenceHSM::state_low_throttle);
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

void FiringSequenceHSM::updateAndLogStatus(FiringSequenceState state)
{
    this->state = state;

    // printf("FiringSequence state updated to %s\n", to_string(state).c_str());
    FiringSequenceData data = {TimestampTimer::getTimestamp(), state};
    sdLogger.log(data);
}

}  // namespace Motor
