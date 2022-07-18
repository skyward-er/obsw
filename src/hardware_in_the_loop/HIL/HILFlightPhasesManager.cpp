/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

HILFlightPhasesManager::HILFlightPhasesManager()
    : counter_flight_events(sEventBroker), counter_airbrakes(sEventBroker),
      counter_ada(sEventBroker), counter_dpl(sEventBroker)
{
    updateFlags({0, 0, 0, 0, 0, 0});

    // it was TOPIC_FLIGHT_EVENTS
    counter_flight_events.subscribe(Main::TOPIC_FLIGHT);
    counter_airbrakes.subscribe(Main::TOPIC_ABK);
    counter_ada.subscribe(Main::TOPIC_ADA);
    counter_dpl.subscribe(Main::TOPIC_DPL);
}

void HILFlightPhasesManager::setCurrentPositionSource(
    std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition)
{
    this->getCurrentPosition = getCurrentPosition;
}

bool HILFlightPhasesManager::isSimulationStarted()
{
    return flagsFlightPhases[FlightPhases::SIMULATION_STARTED];
}

bool HILFlightPhasesManager::isSimulationStopped()
{
    return flagsFlightPhases[FlightPhases::SIMULATION_STOPPED];
}

bool HILFlightPhasesManager::isSimulationRunning()
{
    return flagsFlightPhases[FlightPhases::SIMULATION_STARTED] &&
           !flagsFlightPhases[FlightPhases::SIMULATION_STOPPED];
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

    // check if the obsw triggered relevant flight events
    checkEvents();

    std::vector<FlightPhases> changed_flags;

    // set true when the first packet from the simulator arrives
    if (isSetTrue(FlightPhases::SIMULATION_STARTED))
    {
        t_start = Boardcore::TimestampTimer::getTimestamp();

        TRACE("[HIL] ------- SIMULATION STARTED ! ------- \n");
        changed_flags.push_back(FlightPhases::SIMULATION_STARTED);
    }

    if (isSetTrue(FlightPhases::CALIBRATION))
    {
        TRACE("[HIL] ------- CALIBRATION ! ------- \n");
        changed_flags.push_back(FlightPhases::CALIBRATION);
    }

    if (isSetFalse(FlightPhases::CALIBRATION))  // calibration finalized
    {
        TRACE("[HIL] ------- READY TO LAUNCH ! ------- \n");
    }

    if (isSetTrue(FlightPhases::LIFTOFF_PIN_DETACHED))
    {
        TRACE("[HIL] ------- LIFTOFF PIN DETACHED ! ------- \n");
        changed_flags.push_back(FlightPhases::LIFTOFF_PIN_DETACHED);
    }

    if (flagsFlightPhases[FlightPhases::FLYING])
    {
        if (isSetTrue(FlightPhases::FLYING))
        {
            t_liftoff = Boardcore::TimestampTimer::getTimestamp();
            sEventBroker.post({EV_UMBILICAL_DETACHED}, TOPIC_FLIGHT_EVENTS);

            TRACE("[HIL] ------- LIFTOFF ! ------- \n");
            changed_flags.push_back(FlightPhases::FLYING);

            TRACE("[HIL] ------- ASCENT ! ------- \n");
            changed_flags.push_back(FlightPhases::ASCENT);
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
        if (isSetTrue(FlightPhases::AEROBRAKES))
        {
            registerOutcomes(FlightPhases::AEROBRAKES);
            TRACE("[HIL] ------- AEROBRAKES ENABLED ! ------- \n");
            changed_flags.push_back(FlightPhases::AEROBRAKES);
        }
        if (isSetTrue(FlightPhases::APOGEE))
        {
            registerOutcomes(FlightPhases::APOGEE);
            TRACE("[HIL] ------- APOGEE DETECTED ! ------- \n");
            changed_flags.push_back(FlightPhases::APOGEE);
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
    else if (isSetTrue(FlightPhases::SIMULATION_STOPPED))
    {
        changed_flags.push_back(FlightPhases::SIMULATION_STOPPED);
        t_stop = Boardcore::TimestampTimer::getTimestamp();
        TRACE("[HIL] ------- SIMULATION STOPPED ! -------: %f \n\n\n",
              (double)t_stop / 1000000.0f);
        printOutcomes();
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
    TRACE("OUTCOMES: (times dt from liftoff)\n\n");
    TRACE("Simulation time: %.3f [sec]\n\n",
          (double)(t_stop - t_start) / 1000000.0f);

    TRACE("Motor stopped burning (simulation flag): \n");
    outcomes[FlightPhases::BURNING].print(t_liftoff);

    TRACE("Airbrakes exit shadowmode: \n");
    outcomes[FlightPhases::AEROBRAKES].print(t_liftoff);

    TRACE("Apogee: \n");
    outcomes[FlightPhases::APOGEE].print(t_liftoff);

    TRACE("Parachute 1: \n");
    outcomes[FlightPhases::PARA1].print(t_liftoff);

    TRACE("Parachute 2: \n");
    outcomes[FlightPhases::PARA2].print(t_liftoff);
}

/**
 * @brief Updates the flags of the object with the flags sent from matlab
 * and checks for the apogee
 */
void HILFlightPhasesManager::updateFlags(FlightPhasesFlags hil_flags)
{
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

void HILFlightPhasesManager::checkEvents()
{
    // calibration flag set to true at the beginning of the simulation
    if (!prev_flagsFlightPhases[FlightPhases::CALIBRATION] &&
        counter_flight_events.getCount(EV_CALIBRATION_OK) == 0 &&
        counter_flight_events.getCount(EV_TC_CALIBRATE_SENSORS) > 0)
    {
        setFlagFlightPhase(FlightPhases::CALIBRATION, true);
    }

    // calibration flag turned false when calibration finishes
    if (prev_flagsFlightPhases[FlightPhases::CALIBRATION] &&
        counter_flight_events.getCount(EV_CALIBRATION_OK) > 0)
    {
        setFlagFlightPhase(FlightPhases::CALIBRATION, false);
    }

    // airbrakes flag turned true when the shadow mode ended
    if (!prev_flagsFlightPhases[FlightPhases::AEROBRAKES] &&
        counter_airbrakes.getCount(EV_SHADOW_MODE_TIMEOUT) > 0)
    {
        setFlagFlightPhase(FlightPhases::AEROBRAKES, true);
    }

    // apogee flag turned true when apogee is detected
    if (!prev_flagsFlightPhases[FlightPhases::APOGEE] &&
        counter_ada.getCount(EV_ADA_APOGEE_DETECTED) > 0)
    {
        setFlagFlightPhase(FlightPhases::APOGEE, true);
    }

    // parachute 1 flag turned true when apogee is detected
    if (!prev_flagsFlightPhases[FlightPhases::PARA1] &&
        counter_dpl.getCount(EV_NC_OPEN) > 0)
    {
        setFlagFlightPhase(FlightPhases::PARA1, true);
    }

    // parachute 2 flag turned true with event deployment altitude detected
    if (!prev_flagsFlightPhases[FlightPhases::PARA2] &&
        counter_ada.getCount(EV_ADA_DPL_ALT_DETECTED) > 0)
    {
        setFlagFlightPhase(FlightPhases::PARA2, true);
    }
}

bool HILFlightPhasesManager::isSetTrue(FlightPhases phase)
{
    return flagsFlightPhases[phase] == true &&
           prev_flagsFlightPhases[phase] == false;
}

bool HILFlightPhasesManager::isSetFalse(FlightPhases phase)
{
    return flagsFlightPhases[phase] == false &&
           prev_flagsFlightPhases[phase] == true;
}