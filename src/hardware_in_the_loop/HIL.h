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

#include "HIL_sensors/HILSensors.h"
#include "NavigationSystem/NASData.h"
#include "hardware_in_the_loop/HILFlightPhasesManager.h"
#include "simulator_communication/HILTransceiver.h"

/**
 * @brief Single interface to the hardware-in-the-loop framework.
 */
class HIL : public Singleton<HIL>
{
    friend class Singleton<HIL>;

public:
    HILFlightPhasesManager* flightPhasesManager;
    HILTransceiver* simulator;

    /**
     * @brief Start the needed hardware-in-the-loop components.
     */
    bool start() { return simulator->start(); }

    void stop() { simulator->stop(); }

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

    /**
     * @brief method that signals to the simulator that the liftoff pin has
     * signaled the detachment
     */
    void signalLiftoff()
    {
        flightPhasesManager->setFlagFlightPhase(LIFTOFF_PIN_DETACHED, true);
        simulator->setActuatorData(-1);  // start code
    }

    void setNAS(Sensor<DeathStackBoard::NASData>* nas)
    {
        flightPhasesManager->setDataForOutcomes(nas);
    }

private:
    HIL()
    {
        flightPhasesManager = new HILFlightPhasesManager();
        simulator           = new HILTransceiver(flightPhasesManager);
    }
};