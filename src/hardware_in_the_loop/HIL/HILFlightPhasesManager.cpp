/* Copyright (c) 2021-2022 Skyward Experimental Rocketry
 * Authors: Luca Conterio, Emilio Corigliano
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

#include "HILFlightPhasesManager.h"

#include <events/Event.h>

#include "events/EventBroker.h"

/* CPUMeter */
// #include "diagnostic/CpuMeter/CpuMeter.h"

using namespace Common;

HILFlightPhasesManager::HILFlightPhasesManager()
    : Boardcore::EventHandler(),
      flagsFlightPhases({{FlightPhases::SIMULATION_STARTED, false},
                         {FlightPhases::CALIBRATION, false},
                         {FlightPhases::CALIBRATION_OK, false},
                         {FlightPhases::ARMED, false},
                         {FlightPhases::LIFTOFF_PIN_DETACHED, false},
                         {FlightPhases::FLYING, false},
                         {FlightPhases::ASCENT, false},
                         {FlightPhases::BURNING, false},
                         {FlightPhases::AEROBRAKES, false},
                         {FlightPhases::SIM_AEROBRAKES, false},
                         {FlightPhases::APOGEE, false},
                         {FlightPhases::PARA1, false},
                         {FlightPhases::PARA2, false},
                         {FlightPhases::SIMULATION_STOPPED, false}}),
      last_event(0)
{
    prev_flagsFlightPhases = flagsFlightPhases;

    Boardcore::EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    Boardcore::EventBroker::getInstance().subscribe(this, TOPIC_FMM);
    Boardcore::EventBroker::getInstance().subscribe(this, TOPIC_TMTC);
    Boardcore::EventBroker::getInstance().subscribe(this, TOPIC_ABK);
    Boardcore::EventBroker::getInstance().subscribe(this, TOPIC_ADA);
    Boardcore::EventBroker::getInstance().subscribe(this, TOPIC_NAS);
}

void HILFlightPhasesManager::setCurrentPositionSource(
    std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition)
{
    this->getCurrentPosition = getCurrentPosition;
}

bool HILFlightPhasesManager::isFlagActive(FlightPhases flag)
{
    return flagsFlightPhases[flag];
}

void HILFlightPhasesManager::registerToFlightPhase(FlightPhases flag,
                                                   TCallback func)
{
    callbacks[flag].push_back(func);
}

void HILFlightPhasesManager::setFlagFlightPhase(FlightPhases flag,
                                                bool isEnable)
{
    flagsFlightPhases[flag] = isEnable;
}

void HILFlightPhasesManager::processFlags(FlightPhasesFlags hil_flags)
{
    updateFlags(hil_flags);

    std::vector<FlightPhases> changed_flags;

    // set true when the first packet from the simulator arrives
    if (isSetTrue(FlightPhases::SIMULATION_STARTED))
    {
        t_start = Boardcore::TimestampTimer::getTimestamp();

        TRACE("[HIL] ------- SIMULATION STARTED ! ------- \n");
        changed_flags.push_back(FlightPhases::SIMULATION_STARTED);
    }

    if (flagsFlightPhases[FlightPhases::FLYING])
    {
        if (isSetTrue(FlightPhases::FLYING))
        {
            TRACE("[HIL] ------- SIMULATOR LIFTOFF ! ------- \n");
            sEventBroker.post(FLIGHT_LAUNCH_PIN_DETACHED, TOPIC_FLIGHT);
        }
        if (isSetFalse(FlightPhases::BURNING))
        {
            registerOutcomes(FlightPhases::BURNING);
            TRACE("[HIL] ------- STOPPED BURNING ! ------- \n");
            changed_flags.push_back(FlightPhases::BURNING);
        }
        if (isSetTrue(FlightPhases::SIM_AEROBRAKES))
        {
            registerOutcomes(FlightPhases::SIM_AEROBRAKES);
            changed_flags.push_back(FlightPhases::SIM_AEROBRAKES);
        }
        if (isSetTrue(FlightPhases::PARA1))
        {
            registerOutcomes(FlightPhases::PARA1);
            TRACE("[HIL] ------- PARACHUTE 1 ! ------- \n");
            changed_flags.push_back(FlightPhases::PARA1);
        }
        if (isSetTrue(FlightPhases::PARA2))
        {
            registerOutcomes(FlightPhases::PARA2);
            TRACE("[HIL] ------- PARACHUTE 2 ! ------- \n");
            changed_flags.push_back(FlightPhases::PARA2);
        }
    }

    /* calling the callbacks subscribed to the changed flags */
    for (unsigned int i = 0; i < changed_flags.size(); i++)
    {
        std::vector<TCallback> callbacksToCall = callbacks[changed_flags[i]];
        for (unsigned int j = 0; j < callbacksToCall.size(); j++)
        {
            callbacksToCall[j]();
        }
    }

    prev_flagsFlightPhases = flagsFlightPhases;
}

void HILFlightPhasesManager::registerOutcomes(FlightPhases phase)
{
    Boardcore::TimedTrajectoryPoint temp = getCurrentPosition();
    outcomes[phase]                      = Outcomes(temp.z, temp.vz);
}

void HILFlightPhasesManager::printOutcomes()
{
    printf("OUTCOMES: (times dt from liftoff)\n\n");
    printf("Simulation time: %.3f [sec]\n\n",
           (double)(t_stop - t_start) / 1000000.0f);

    printf("Motor stopped burning (simulation flag): \n");
    outcomes[FlightPhases::BURNING].print(t_liftoff);

    printf("Airbrakes exit shadowmode: \n");
    outcomes[FlightPhases::AEROBRAKES].print(t_liftoff);

    printf("Apogee: \n");
    outcomes[FlightPhases::APOGEE].print(t_liftoff);

    printf("Parachute 1: \n");
    outcomes[FlightPhases::PARA1].print(t_liftoff);

    printf("Parachute 2: \n");
    outcomes[FlightPhases::PARA2].print(t_liftoff);

    printf("Simulation Stopped: \n");
    outcomes[FlightPhases::SIMULATION_STOPPED].print(t_liftoff);

    // auto cpuMeter = Boardcore::CpuMeter::getCpuStats();
    // printf("max cpu usage: %+.3f\n", cpuMeter.maxValue);
    // printf("avg cpu usage: %+.3f\n", cpuMeter.mean);
    // printf("min free heap: %+.3lu\n", cpuMeter.minFreeHeap);
    // printf("max free stack:%+.3lu\n", cpuMeter.minFreeStack);
}

/**
 * @brief Updates the flags of the object with the flags sent from matlab
 * and checks for the apogee
 */
void HILFlightPhasesManager::updateFlags(FlightPhasesFlags hil_flags)
{
    // [TODO] add ifdefs in order to select which flags to take from matlab
    flagsFlightPhases[FlightPhases::ASCENT]  = hil_flags.flag_ascent;
    flagsFlightPhases[FlightPhases::FLYING]  = hil_flags.flag_flight;
    flagsFlightPhases[FlightPhases::BURNING] = hil_flags.flag_burning;

    /* Flags PARA1, PARA2 and SIM_AEROBRAKES ignored from matlab  */
    // flagsFlightPhases[SIM_AEROBRAKES] = hil_flags.flag_airbrakes;
    // flagsFlightPhases[PARA1]          = hil_flags.flag_para1;
    // flagsFlightPhases[PARA2]          = hil_flags.flag_para2;

    flagsFlightPhases[FlightPhases::SIMULATION_STOPPED] =
        isSetFalse(FlightPhases::FLYING) ||
        prev_flagsFlightPhases[FlightPhases::SIMULATION_STOPPED];
}

void HILFlightPhasesManager::handleEvent(const Boardcore::Event& e)
{
    std::vector<FlightPhases> changed_flags;

    switch (e)
    {
        case FMM_INIT_OK:
            setFlagFlightPhase(FlightPhases::CALIBRATION, true);
            TRACE("[HIL] ------- CALIBRATION ! ------- \n");
            changed_flags.push_back(FlightPhases::CALIBRATION);
            break;
        case NAS_READY:
        case FMM_ALGOS_CAL_DONE:
            setFlagFlightPhase(FlightPhases::CALIBRATION_OK, true);
            TRACE("[HIL] CALIBRATION OK!\n");
            changed_flags.push_back(FlightPhases::CALIBRATION_OK);
            break;
        case FLIGHT_ARMED:
            setFlagFlightPhase(FlightPhases::ARMED, true);
            printf("[HIL] ------- READY TO LAUNCH ! ------- \n");
            changed_flags.push_back(FlightPhases::ARMED);
            break;
        case FLIGHT_LAUNCH_PIN_DETACHED:
            setFlagFlightPhase(FlightPhases::LIFTOFF_PIN_DETACHED, true);
            TRACE("[HIL] ------- LIFTOFF PIN DETACHED ! ------- \n");
            sEventBroker.post(FLIGHT_LIFTOFF, TOPIC_FLIGHT);
            changed_flags.push_back(FlightPhases::LIFTOFF_PIN_DETACHED);
            break;
        case FLIGHT_LIFTOFF:
        case TMTC_FORCE_LAUNCH:
            t_liftoff = Boardcore::TimestampTimer::getTimestamp();
            TRACE("[HIL] ------- LIFTOFF -------: %f, %f \n",
                  getCurrentPosition().z, getCurrentPosition().vz);
            changed_flags.push_back(FlightPhases::FLYING);
            changed_flags.push_back(FlightPhases::ASCENT);
            break;
        case ABK_SHADOW_MODE_TIMEOUT:
            setFlagFlightPhase(FlightPhases::AEROBRAKES, true);
            registerOutcomes(FlightPhases::AEROBRAKES);
            TRACE("[HIL] ------- AEROBRAKES ENABLED ! ------- \n");
            changed_flags.push_back(FlightPhases::AEROBRAKES);
            break;
        case ADA_SHADOW_MODE_TIMEOUT:
            TRACE("[HIL] ADA shadow mode timeout\n");
            break;
        case ABK_DISABLE:
            TRACE("[HIL] ABK disabled\n");
            break;
        case FLIGHT_APOGEE_DETECTED:
        case TMTC_FORCE_EXPULSION:
            setFlagFlightPhase(FlightPhases::AEROBRAKES, false);
            registerOutcomes(FlightPhases::APOGEE);
            TRACE("[HIL] ------- APOGEE DETECTED ! ------- %f, %f \n",
                  getCurrentPosition().z, getCurrentPosition().vz);
            changed_flags.push_back(FlightPhases::APOGEE);
            break;
        case ADA_PRESS_STAB_TIMEOUT:
            setFlagFlightPhase(FlightPhases::PARA1, true);
            registerOutcomes(FlightPhases::PARA1);
            TRACE("[HIL] ------- PARA1 ! -------%f, %f \n",
                  getCurrentPosition().z, getCurrentPosition().vz);
            changed_flags.push_back(FlightPhases::PARA1);
            break;
        case FLIGHT_DPL_ALT_DETECTED:
        case TMTC_FORCE_MAIN:
            setFlagFlightPhase(FlightPhases::PARA1, false);
            setFlagFlightPhase(FlightPhases::PARA2, true);
            registerOutcomes(FlightPhases::PARA2);
            TRACE("[HIL] ------- PARA2 ! ------- %f, %f \n",
                  getCurrentPosition().z, getCurrentPosition().vz);
            changed_flags.push_back(FlightPhases::PARA2);
            break;
        case FLIGHT_LANDING_DETECTED:
        case TMTC_FORCE_LANDING:
            t_stop = Boardcore::TimestampTimer::getTimestamp();
            setFlagFlightPhase(FlightPhases::PARA2, false);
            setFlagFlightPhase(FlightPhases::SIMULATION_STOPPED, true);
            changed_flags.push_back(FlightPhases::SIMULATION_STOPPED);
            registerOutcomes(FlightPhases::SIMULATION_STOPPED);
            TRACE("[HIL] ------- SIMULATION STOPPED ! -------: %f \n\n\n",
                  (double)t_stop / 1000000.0f);
            printOutcomes();
            break;
        default:
            TRACE("%s invalid event\n", getEventString(e).c_str());
    }

    /* calling the callbacks subscribed to the changed flags */
    for (unsigned int i = 0; i < changed_flags.size(); i++)
    {
        std::vector<TCallback> callbacksToCall = callbacks[changed_flags[i]];
        for (unsigned int j = 0; j < callbacksToCall.size(); j++)
        {
            callbacksToCall[j]();
        }
    }

    prev_flagsFlightPhases = flagsFlightPhases;

    last_event = e;
}

bool HILFlightPhasesManager::isSetTrue(FlightPhases phase)
{
    return flagsFlightPhases[phase] && !prev_flagsFlightPhases[phase];
}

bool HILFlightPhasesManager::isSetFalse(FlightPhases phase)
{
    return !flagsFlightPhases[phase] && prev_flagsFlightPhases[phase];
}
