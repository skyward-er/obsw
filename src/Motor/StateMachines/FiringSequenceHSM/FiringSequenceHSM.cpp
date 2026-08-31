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

#include <Motor/Actuators/Actuators.h>
#include <Motor/BoardScheduler.h>
#include <Motor/Configs/FiringSequenceConfig.h>
#include <Motor/PersistentVars/PersistentVars.h>
#include <Motor/Registry/Registry.h>
#include <Motor/Sensors/Sensors.h>
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
    : HSM(&FiringSequenceHSM::state_init, STACK_DEFAULT_FOR_PTHREAD,
          BoardScheduler::firingSequenceHSMPriority())
{
    EventBroker::getInstance().subscribe(this, TOPIC_FIRING_SEQUENCE);
}

void FiringSequenceHSM::setFiringParams(uint32_t fullThrottleTime,
                                        uint32_t lowThrottleTime,
                                        uint32_t pilotLeadTime,
                                        float pilotFlameOxPosition,
                                        float pilotFlameFuelPosition)
{
    getModule<Registry>()->setUnsafe(CONFIG_ID_FULL_THROTTLE_TIME,
                                     fullThrottleTime);
    getModule<Registry>()->setUnsafe(CONFIG_ID_LOW_THROTTLE_TIME,
                                     lowThrottleTime);
    getModule<Registry>()->setUnsafe(CONFIG_ID_PILOT_LEAD_TIME, pilotLeadTime);
    getModule<Registry>()->setUnsafe(CONFIG_ID_PILOT_FLAME_OX_POSITION,
                                     pilotFlameOxPosition);
    getModule<Registry>()->setUnsafe(CONFIG_ID_PILOT_FLAME_FUEL_POSITION,
                                     pilotFlameFuelPosition);

    EventBroker::getInstance().post(FIRING_SEQUENCE_CONFIG_SET,
                                    TOPIC_FIRING_SEQUENCE);
}

void FiringSequenceHSM::setPressureThresholds(float igniterThreshold,
                                              float pilotFlameThreshold)
{
    getModule<Registry>()->setUnsafe(CONFIG_ID_IGNITER_PRESSURE_THRESHOLD,
                                     igniterThreshold);
    getModule<Registry>()->setUnsafe(CONFIG_ID_PILOT_FLAME_PRESSURE_THRESHOLD,
                                     pilotFlameThreshold);

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

    przTankPressureThreshold = getModule<Registry>()->getOrSetDefaultUnsafe(
        CONFIG_ID_PRZ_TANK_PRESSURE_THRESHOLD,
        Config::FiringSequence::PRZ_TANK_PRESSURE_THRESHOLD);

    oxTankPressureThreshold = getModule<Registry>()->getOrSetDefaultUnsafe(
        CONFIG_ID_OX_TANK_PRESSURE_THRESHOLD,
        Config::FiringSequence::OX_TANK_PRESSURE_THRESHOLD);

    igniterTaskId = scheduler.addTask([this]() { checkIgniterPressure(); },
                                      Config::FiringSequence::UPDATE_RATE);

    pilotFlameTaskId =
        scheduler.addTask([this]() { checkPilotFlamePressure(); },
                          Config::FiringSequence::UPDATE_RATE);

    depressurizationTaskId =
        scheduler.addTask([this]() { checkDepressurizationPressure(); },
                          Config::FiringSequence::UPDATE_RATE);

    if (igniterTaskId == 0 || pilotFlameTaskId == 0 ||
        depressurizationTaskId == 0)
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

void FiringSequenceHSM::checkDepressurizationPressure()
{
    auto now = steady_clock::now();

    if (state == FiringSequenceState::DEPRESSURIZATION_OX)
    {
        if (getModule<Sensors>()->getOxTankPressure().pressure >=
            oxTankPressureThreshold)
            lastPressureOverTime = now;
        if (now - lastPressureOverTime >=
            Config::FiringSequence::Depressurization::OX_HYSTERESIS)
        {
            EventBroker::getInstance().post(
                FIRING_SEQUENCE_DEPRESSURIZATION_OX_DONE,
                TOPIC_FIRING_SEQUENCE);
        }
    }
    else if (state == FiringSequenceState::DEPRESSURIZATION_PRZ)
    {
        if (getModule<Sensors>()->getPrzTankPressure().pressure >=
            przTankPressureThreshold)
            lastPressureOverTime = now;
        if (now - lastPressureOverTime >=
            Config::FiringSequence::Depressurization::PRZ_OX_HYSTERESIS)
        {
            EventBroker::getInstance().post(
                FIRING_SEQUENCE_DEPRESSURIZATION_PRZ_DONE,
                TOPIC_FIRING_SEQUENCE);
        }
    }
}

State FiringSequenceHSM::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FiringSequenceState::INIT);
            return HANDLED;
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case FIRING_SEQUENCE_CONFIG_SET:
        {
            return transition(&FiringSequenceHSM::state_ready);
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

State FiringSequenceHSM::state_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FiringSequenceState::READY);
            getModule<Actuators>()->closeAllValves();
            return HANDLED;
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case FIRING_SEQUENCE_START:
        {
            return transition(&FiringSequenceHSM::state_firing);
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

State FiringSequenceHSM::state_firing(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FiringSequenceState::FIRING);

            return HANDLED;
        }

        case EV_INIT:
        {
            return transition(&FiringSequenceHSM::state_igniter);
        }

        case FIRING_SEQUENCE_END:
        case FIRING_SEQUENCE_ABORT:
        {
            EventBroker::getInstance().removeDelayed(nextEventId);
            return transition(&FiringSequenceHSM::state_ended);
        }

        case EV_EMPTY:
        {
            return tranSuper(&FiringSequenceHSM::state_top);
        }

        case EV_EXIT:
        {
            getModule<Actuators>()->closeAllValves();
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

        case EV_INIT:
        {
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

        case EV_EMPTY:
        {
            return tranSuper(&FiringSequenceHSM::state_firing);
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

            // Reset igniter confirmation samples
            igniterFlameSamples = 0;
            return HANDLED;
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case FIRING_SEQUENCE_IGNITER_OK:
        {
            return transition(&FiringSequenceHSM::state_pilot_flame);
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

            return HANDLED;
        }

        case EV_INIT:
        {
            return transition(&FiringSequenceHSM::state_pilot_flame_wait);
        }

        case EV_EMPTY:
        {
            return tranSuper(&FiringSequenceHSM::state_firing);
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

            nextEventId = EventBroker::getInstance().postDelayed(
                FIRING_SEQUENCE_PILOT_FLAME_TIMEOUT, TOPIC_FIRING_SEQUENCE,
                Config::FiringSequence::PILOT_FLAME_MAX_TIME.count());

            // Reset pilot flame confirmation samples
            pilotFlameSamples = 0;

            return HANDLED;
        }

        case EV_INIT:
        {
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

        case EV_INIT:
        {
            return HANDLED;
        }

        case FIRING_SEQUENCE_FULL_THROTTLE:
        {
            return transition(&FiringSequenceHSM::state_full_throttle);
        }

        case EV_EMPTY:
        {
            return tranSuper(&FiringSequenceHSM::state_firing);
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

        case EV_INIT:
        {
            return HANDLED;
        }

        case FIRING_SEQUENCE_LOW_THROTTLE:
        {
            return transition(&FiringSequenceHSM::state_low_throttle);
        }

        case EV_EMPTY:
        {
            return tranSuper(&FiringSequenceHSM::state_firing);
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

            // retreive params from registry
            uint32_t lowThrottleTime =
                getModule<Registry>()->getOrSetDefaultUnsafe(
                    CONFIG_ID_LOW_THROTTLE_TIME,
                    static_cast<uint32_t>(
                        Config::FiringSequence::LOW_THROTTLE_TIME.count()));

            // move valves to low throttle position
            getModule<Actuators>()->moveValve(
                ServosList::MAIN_OX_VALVE,
                Config::FiringSequence::LOW_THROTTLE_OX_POSITION);

            getModule<Actuators>()->moveValve(
                ServosList::MAIN_FUEL_VALVE,
                Config::FiringSequence::LOW_THROTTLE_FUEL_POSITION);

            nextEventId = EventBroker::getInstance().postDelayed(
                FIRING_SEQUENCE_END, TOPIC_FIRING_SEQUENCE, lowThrottleTime);

            return HANDLED;
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&FiringSequenceHSM::state_firing);
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
            bool hil = PersistentVars::getHilMode();
            updateAndLogStatus(FiringSequenceState::ENDED);

            // If not HIL then stop MEA. If HIL then it will be stopped by the
            // Firing Sequence received by CHAD
            if (!hil)
                EventBroker::getInstance().post(MEA_STOP, TOPIC_MEA);

            EventBroker::getInstance().post(EREG_CLOSE, TOPIC_EREG_OX);
            EventBroker::getInstance().post(EREG_CLOSE, TOPIC_EREG_FUEL);

            getModule<Actuators>()->closeValve(ServosList::MAIN_OX_VALVE);
            getModule<Actuators>()->closeValve(ServosList::MAIN_FUEL_VALVE);

            return HANDLED;
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case FIRING_SEQUENCE_ABORT:
        {
            return transition(&FiringSequenceHSM::state_ready);
        }

        case CAN_APOGEE_DETECTED:
        {
            return transition(&FiringSequenceHSM::state_depressurization_ox);
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

State FiringSequenceHSM::state_depressurization_ox(const Event& event)
{
    using namespace Config::FiringSequence::Depressurization;
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FiringSequenceState::DEPRESSURIZATION_OX);
            getModule<Actuators>()->openValveWithTime(
                ServosList::OX_VENTING_VALVE,
                milliseconds{OX_VENTING_TIMEOUT}.count());
            lastPressureOverTime = steady_clock::now();
            nextEventId          = EventBroker::getInstance().postDelayed(
                FIRING_SEQUENCE_DEPRESSURIZATION_OX_DONE, TOPIC_FIRING_SEQUENCE,
                milliseconds{OX_VENTING_TIMEOUT}.count());
            return HANDLED;
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case FIRING_SEQUENCE_DEPRESSURIZATION_OX_DONE:
        {
            EventBroker::getInstance().removeDelayed(nextEventId);
            return transition(&FiringSequenceHSM::state_depressurization_prz);
        }

        case FIRING_SEQUENCE_ABORT:
        {
            EventBroker::getInstance().removeDelayed(nextEventId);
            return transition(&FiringSequenceHSM::state_ready);
        }

        case EV_EMPTY:
        {
            return tranSuper(&FiringSequenceHSM::state_ended);
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

State FiringSequenceHSM::state_depressurization_prz(const Event& event)
{
    using namespace Config::FiringSequence::Depressurization;
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FiringSequenceState::DEPRESSURIZATION_PRZ);

            getModule<Actuators>()->moveValve(ServosList::PRZ_OX_VALVE,
                                              PRZ_OX_APERTURE);
            lastPressureOverTime = steady_clock::now();
            nextEventId          = EventBroker::getInstance().postDelayed(
                FIRING_SEQUENCE_DEPRESSURIZATION_PRZ_DONE,
                TOPIC_FIRING_SEQUENCE, milliseconds{PRZ_OX_TIMEOUT}.count());

            return HANDLED;
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case FIRING_SEQUENCE_DEPRESSURIZATION_PRZ_DONE:
        {
            EventBroker::getInstance().removeDelayed(nextEventId);
            return transition(&FiringSequenceHSM::state_depressurization_fuel);
        }
        case FIRING_SEQUENCE_ABORT:
        {
            EventBroker::getInstance().removeDelayed(nextEventId);
            return transition(&FiringSequenceHSM::state_ready);
        }
        case EV_EMPTY:
        {
            return tranSuper(&FiringSequenceHSM::state_ended);
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

State FiringSequenceHSM::state_depressurization_fuel(const Event& event)
{
    using namespace Config::FiringSequence::Depressurization;
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FiringSequenceState::DEPRESSURIZATION_FUEL);

            getModule<Actuators>()->openValveWithTime(
                ServosList::PRZ_FUEL_VALVE,
                milliseconds{PRZ_FUEL_TIMEOUT}.count());
            nextEventId = EventBroker::getInstance().postDelayed(
                FIRING_SEQUENCE_DEPRESSURIZATION_FUEL_DONE,
                TOPIC_FIRING_SEQUENCE, milliseconds{PRZ_FUEL_TIMEOUT}.count());

            return HANDLED;
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case FIRING_SEQUENCE_DEPRESSURIZATION_FUEL_DONE:
        {
            return transition(&FiringSequenceHSM::state_depressurization_done);
        }
        case FIRING_SEQUENCE_ABORT:
        {
            EventBroker::getInstance().removeDelayed(nextEventId);
            return transition(&FiringSequenceHSM::state_ready);
        }
        case EV_EMPTY:
        {
            return tranSuper(&FiringSequenceHSM::state_ended);
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
State FiringSequenceHSM::state_depressurization_done(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FiringSequenceState::DEPRESSURIZATION_DONE);

            // Disable tasks, as we don't need to check the pressure anymore
            getModule<BoardScheduler>()->firingSequenceHSM().disableTask(
                depressurizationTaskId);
            getModule<BoardScheduler>()->firingSequenceHSM().disableTask(
                igniterTaskId);
            getModule<BoardScheduler>()->firingSequenceHSM().disableTask(
                pilotFlameTaskId);

            // Should we close the valves here? Maybe we should leave them open
            // for a while to ensure depressurization is complete. For now,
            // let's close them.

            return HANDLED;
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&FiringSequenceHSM::state_ended);
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

    FiringSequenceData data = {TimestampTimer::getTimestamp(), state};
    sdLogger.log(data);
}

}  // namespace Motor
