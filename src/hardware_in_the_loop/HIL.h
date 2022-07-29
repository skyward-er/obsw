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

#include "HIL/HILFlightPhasesManager.h"
#include "HIL/HILTransceiver.h"

/**
 * @brief Single interface to the hardware-in-the-loop framework.
 */
class HIL : public Boardcore::Singleton<HIL>
{
    friend class Boardcore::Singleton<HIL>;

public:
    HILTransceiver *simulator;
    HILFlightPhasesManager *flightPhasesManager;

    /**
     * @brief Start the needed hardware-in-the-loop components.
     */
    bool start() { return simulator->start(); }

    void stop() { simulator->stop(); }

    void send(ActuatorData d) { simulator->setActuatorData(d); }

    /**
     * @brief method that signals to the simulator that the liftoff pin has
     * signaled the detachment
     */
    void signalLiftoff()
    {
        flightPhasesManager->setFlagFlightPhase(
            FlightPhases::LIFTOFF_PIN_DETACHED, true);

        // start code for the flight
        actuatorData.setAirBrakesOpening(-1);
        simulator->setActuatorData(actuatorData);
    }

    bool isSimulationStarted()
    {
        return flightPhasesManager->isSimulationStarted();
    }

    bool isSimulationStopped()
    {
        return flightPhasesManager->isSimulationStopped();
    }

    bool isSimulationRunning()
    {
        return flightPhasesManager->isSimulationRunning();
    }

    ActuatorData *getActuatorData() { return &actuatorData; }

private:
    HIL()
    {
        flightPhasesManager = new HILFlightPhasesManager();
        simulator           = new HILTransceiver(flightPhasesManager);
    }

    ActuatorData actuatorData;
};
