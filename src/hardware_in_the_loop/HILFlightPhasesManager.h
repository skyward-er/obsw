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

#pragma once

#include <events/Events.h>
#include <events/utils/EventCounter.h>

#include <iostream>
#include <map>

#include "ActiveObject.h"
#include "Algorithm.h"
#include "HIL_sensors/HILSensors.h"
#include "NavigationAttitudeSystem/NASData.h"
#include "Singleton.h"
#include "TimestampTimer.h"
#include "hardware_in_the_loop/HILConfig.h"
#include "miosix.h"
#include "sensors/Sensor.h"

using namespace miosix;
using namespace std;

typedef function<void()> TCallback;

enum FlightPhases
{
    SIMULATION_STARTED,
    CALIBRATION,
    LIFTOFF_PIN_DETACHED,
    FLYING,
    ASCENT,
    BURNING,
    AEROBRAKES,
    SIM_AEROBRAKES,
    APOGEE,
    PARA1,
    PARA2,
    SIMULATION_STOPPED
};

struct Outcomes
{
    uint64_t t = 0;
    float z    = 0;
    float vz   = 0;

    Outcomes() : t(0), z(0), vz(0) {}
    Outcomes(float z, float vz)
        : t(TimestampTimer::getTimestamp()), z(z), vz(vz)
    {
    }

    void print(uint64_t t_start) const
    {
        TRACE("@time     : %f [sec]\n", (double)(t - t_start) / 1000000);
        TRACE("@altitude : %f [m]\n", z);
        TRACE("@velocity : %f [m/s]\n\n", vz);
    }
};

/**
 * @brief Singleton object that manages all the phases of the simulation
 */
class HILFlightPhasesManager
{
    using FlightPhasesFlags = SimulatorData::Flags;

public:
    HILFlightPhasesManager()
    {
        updateFlags({0, 0, 0, 0, 0, 0});
        counter_flight_events = new EventCounter(*sEventBroker);
        counter_flight_events->subscribe(TOPIC_FLIGHT_EVENTS);

        counter_airbrakes = new EventCounter(*sEventBroker);
        counter_airbrakes->subscribe(TOPIC_ABK);

        counter_ada = new EventCounter(*sEventBroker);
        counter_ada->subscribe(TOPIC_ADA);

        counter_dpl = new EventCounter(*sEventBroker);
        counter_dpl->subscribe(TOPIC_DPL);
    }

    bool isSimulationStarted() { return flagsFlightPhases[SIMULATION_STARTED]; }

    bool isSimulationStopped() { return flagsFlightPhases[SIMULATION_STOPPED]; }

    bool isSimulationRunning()
    {
        return flagsFlightPhases[SIMULATION_STARTED] &&
               !flagsFlightPhases[SIMULATION_STOPPED];
    }

    void registerToFlightPhase(FlightPhases flag, TCallback func)
    {
        callbacks[flag].push_back(func);
    }

    void setFlagFlightPhase(FlightPhases flag, bool isEnable)
    {
        flagsFlightPhases[flag] = isEnable;
    }

    void processFlags(FlightPhasesFlags hil_flags)
    {
        updateFlags(hil_flags);

        // check if the obsw triggered relevant flight events
        checkEvents();

        vector<FlightPhases> changed_flags;

        // set true when the first packet from the simulator arrives
        if (isSetTrue(SIMULATION_STARTED))
        {
            t_start = TimestampTimer::getTimestamp();

            TRACE("[HIL] ------- SIMULATION STARTED ! ------- \n");
            changed_flags.push_back(SIMULATION_STARTED);
        }

        if (isSetTrue(CALIBRATION))
        {
            TRACE("[HIL] ------- CALIBRATION ! ------- \n");
            changed_flags.push_back(CALIBRATION);
        }

        if (isSetFalse(CALIBRATION))  // calibration finalized
        {
            TRACE("[HIL] ------- READY TO LAUNCH ! ------- \n");
        }

        if (isSetTrue(LIFTOFF_PIN_DETACHED))
        {
            TRACE("[HIL] ------- LIFTOFF PIN DETACHED ! ------- \n");
            changed_flags.push_back(LIFTOFF_PIN_DETACHED);
        }

        if (flagsFlightPhases[FLYING])
        {
            if (isSetTrue(FLYING))
            {
                t_liftoff = TimestampTimer::getTimestamp();
                sEventBroker->post({EV_UMBILICAL_DETACHED},
                                   TOPIC_FLIGHT_EVENTS);

                TRACE("[HIL] ------- LIFTOFF ! ------- \n");
                changed_flags.push_back(FLYING);

                TRACE("[HIL] ------- ASCENT ! ------- \n");
                changed_flags.push_back(ASCENT);
            }
            if (isSetFalse(BURNING))
            {
                registerOutcomes(BURNING);
                TRACE("[HIL] ------- STOPPED BURNING ! ------- \n");
                changed_flags.push_back(BURNING);
            }
            if (isSetTrue(SIM_AEROBRAKES))
            {
                registerOutcomes(SIM_AEROBRAKES);
                changed_flags.push_back(SIM_AEROBRAKES);
            }
            if (isSetTrue(AEROBRAKES))
            {
                registerOutcomes(AEROBRAKES);
                TRACE("[HIL] ------- AEROBRAKES ENABLED ! ------- \n");
                changed_flags.push_back(AEROBRAKES);
            }
            if (isSetTrue(APOGEE))
            {
                registerOutcomes(APOGEE);
                TRACE("[HIL] ------- APOGEE DETECTED ! ------- \n");
                changed_flags.push_back(APOGEE);
            }
            if (isSetTrue(PARA1))
            {
                registerOutcomes(PARA1);
                TRACE("[HIL] ------- PARACHUTE 1 ! ------- \n");
                changed_flags.push_back(PARA1);
            }
            if (isSetTrue(PARA2))
            {
                registerOutcomes(PARA2);
                TRACE("[HIL] ------- PARACHUTE 2 ! ------- \n");
                changed_flags.push_back(PARA2);
            }
        }
        else if (isSetTrue(SIMULATION_STOPPED))
        {
            changed_flags.push_back(SIMULATION_STOPPED);
            t_stop = TimestampTimer::getTimestamp();
            TRACE("[HIL] ------- SIMULATION STOPPED ! -------: %f \n\n\n",
                  (double)t_stop / 1000000.0f);
            printOutcomes();
        }

        /* calling the callbacks subscribed to the changed flags */
        for (unsigned int i = 0; i < changed_flags.size(); i++)
        {
            vector<TCallback> callbacksToCall = callbacks[changed_flags[i]];
            for (unsigned int j = 0; j < callbacksToCall.size(); j++)
            {
                callbacksToCall[j]();
            }
        }

        prev_flagsFlightPhases = flagsFlightPhases;
    }

    void setSourceForOutcomes(Sensor<DeathStackBoard::NASData>* nas)
    {
        this->nas = nas;
    }

private:
    void registerOutcomes(FlightPhases phase)
    {
        outcomes[phase] =
            Outcomes(nas->getLastSample().z, nas->getLastSample().vz);
    }

    void printOutcomes()
    {
        TRACE("OUTCOMES: (times dt from liftoff)\n\n");
        TRACE("Simulation time: %.3f [sec]\n\n",
              (double)(t_stop - t_start) / 1000000.0f);

        TRACE("Motor stopped burning (simulation flag): \n");
        outcomes[BURNING].print(t_liftoff);

        TRACE("Airbrakes exit shadowmode: \n");
        outcomes[AEROBRAKES].print(t_liftoff);

        TRACE("Apogee: \n");
        outcomes[APOGEE].print(t_liftoff);

        TRACE("Parachute 1: \n");
        outcomes[PARA1].print(t_liftoff);

        TRACE("Parachute 2: \n");
        outcomes[PARA2].print(t_liftoff);
    }

    /**
     * @brief Updates the flags of the object with the flags sent from matlab
     * and checks for the apogee
     */
    void updateFlags(FlightPhasesFlags hil_flags)
    {
        flagsFlightPhases[ASCENT]  = hil_flags.flag_ascent;
        flagsFlightPhases[FLYING]  = hil_flags.flag_flight;
        flagsFlightPhases[BURNING] = hil_flags.flag_burning;

        /* Flags PARA1, PARA2 and SIM_AEROBRAKES ignored from matlab  */
        // flagsFlightPhases[SIM_AEROBRAKES] = hil_flags.flag_airbrakes;
        // flagsFlightPhases[PARA1]          = hil_flags.flag_para1;
        // flagsFlightPhases[PARA2]          = hil_flags.flag_para2;

        flagsFlightPhases[SIMULATION_STOPPED] =
            isSetFalse(FLYING) || prev_flagsFlightPhases[SIMULATION_STOPPED];
    }

    void checkEvents()
    {
        // calibration flag set to true at the beginning of the simulation
        if (!prev_flagsFlightPhases[CALIBRATION] &&
            counter_flight_events->getCount(EV_CALIBRATION_OK) == 0 &&
            counter_flight_events->getCount(EV_TC_CALIBRATE_SENSORS) > 0)
        {
            setFlagFlightPhase(CALIBRATION, true);
        }

        // calibration flag turned false when calibration finishes
        if (prev_flagsFlightPhases[CALIBRATION] &&
            counter_flight_events->getCount(EV_CALIBRATION_OK) > 0)
        {
            setFlagFlightPhase(CALIBRATION, false);
        }

        // airbrakes flag turned true when the shadow mode ended
        if (!prev_flagsFlightPhases[AEROBRAKES] &&
            counter_airbrakes->getCount(EV_SHADOW_MODE_TIMEOUT) > 0)
        {
            setFlagFlightPhase(AEROBRAKES, true);
        }

        // apogee flag turned true when apogee is detected
        if (!prev_flagsFlightPhases[APOGEE] &&
            counter_ada->getCount(EV_ADA_APOGEE_DETECTED) > 0)
        {
            setFlagFlightPhase(APOGEE, true);
        }

        // parachute 1 flag turned true when apogee is detected
        if (!prev_flagsFlightPhases[PARA1] &&
            counter_dpl->getCount(EV_NC_OPEN) > 0)
        {
            setFlagFlightPhase(PARA1, true);
        }

        // parachute 2 flag turned true with event deployment altitude detected
        if (!prev_flagsFlightPhases[PARA2] &&
            counter_ada->getCount(EV_ADA_DPL_ALT_DETECTED) > 0)
        {
            setFlagFlightPhase(PARA2, true);
        }
    }

    bool isSetTrue(FlightPhases phase)
    {
        return flagsFlightPhases[phase] == true &&
               prev_flagsFlightPhases[phase] == false;
    }

    bool isSetFalse(FlightPhases phase)
    {
        return flagsFlightPhases[phase] == false &&
               prev_flagsFlightPhases[phase] == true;
    }

    uint64_t t_start   = 0;
    uint64_t t_liftoff = 0;
    uint64_t t_stop    = 0;
    map<FlightPhases, bool> flagsFlightPhases;
    map<FlightPhases, bool> prev_flagsFlightPhases;
    map<FlightPhases, vector<TCallback>> callbacks;
    map<FlightPhases, Outcomes> outcomes;
    Sensor<DeathStackBoard::NASData>* nas;

    EventCounter* counter_flight_events;
    EventCounter* counter_airbrakes;
    EventCounter* counter_ada;
    EventCounter* counter_dpl;
};