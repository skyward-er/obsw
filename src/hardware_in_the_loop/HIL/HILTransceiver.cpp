/* Copyright (c) 2020-2021 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include "HIL.h"

#ifdef PAYLOAD_ENTRY
#include <Payload/Buses.h>
using namespace Payload;
#else
#include <Main/Buses.h>
using namespace Main;
#endif

/**
 * @brief Construct a serial connection attached to a control algorithm
 */
HILTransceiver::HILTransceiver()
    : hilSerial(Buses::getInstance().uart4), actuatorData(ActuatorData{})
{
}

/**
 * @brief sets the actuator data and then wakes up the MatlabTransceiver
 * thread in order to send the data back to the simulator (called by the
 * control algorithm)
 * @param actuatorData sets the data that will be sent to the simulator
 */
void HILTransceiver::setActuatorData(ActuatorData actuatorData)
{
    this->actuatorData = actuatorData;
    updated            = true;
    condVar.signal();
}

/**
 * @brief returns the reference of the SimulatorData
 *
 * @return reference to the data simulated by matlab
 */
SimulatorData *HILTransceiver::getSensorData() { return &sensorData; }

/**
 * @brief adds to the resetSampleCounter list an object that has to be
 * notified when a new packet of data is arrived from the simulator
 *
 * @param t SimTimestampManagement object
 */
void HILTransceiver::addResetSampleCounter(HILTimestampManagement *t)
{
    sensorsTimestamp.push_back(t);
}

/**
 * @brief adds to the notifyToBegin list an object that has to be started
 * when the first packet from the simulator arrives.
 *
 * @param algorithm Algorithm object to be started
 */
/*void setIsAirbrakePhase(bool isAirbrakePhase)
{
    this->isAirbrakePhase = isAirbrakePhase;
    if (!isAirbrakePhase)
    {
        actuatorData = 0;
    }
}*/

/**
 * @brief The thread deals with the communication between the simulator and the
 * board.
 *
 * TODO: Check:
 * The first read is done in the init() function
 *
 * After the first time the data is received, the loop of this thread:
 * - Reads the simulated data and copies them in the SensorData structure;
 * - Notifies every sensor that new data arrived;
 * - Waits for the control algorithms to update the actuator data;
 * - Sends back the value to the simulator.
 */
void HILTransceiver::run()
{
    TRACE("[HILT] Transceiver started\n");
    bool lostUpdate = false;
    HILFlightPhasesManager *flightPhasesManager =
        HIL::getInstance().flightPhasesManager;

    while (true)
    {
        // Pausing the kernel in order to copy the data in the shared structure
        {
            SimulatorData tempData;
            hilSerial.read(&tempData, sizeof(SimulatorData));

            miosix::PauseKernelLock kLock;
            sensorData = tempData;

            if (updated)
            {
                lostUpdate = true;
                updated    = false;  // We want the last computation
            }
        }

        // If this is the first packet to be received, then update the flight
        // phase manager
        if (!receivedFirstPacket)
        {
            receivedFirstPacket = true;
            flightPhasesManager->setFlagFlightPhase(
                FlightPhases::SIMULATION_STARTED, true);
        }

        // Notify all sensors that a new set of data is arrived
        //[REVIEW] Could be moved in HILFlightPhasesManager
        for (auto st : sensorsTimestamp)
            st->resetSampleCounter();

        // Trigger events relative to the flight phases
        flightPhasesManager->processFlags(sensorData.flags);

        if (lostUpdate)
        {
            // This means also that the number of samples used for the mean sent
            // to the HIL simulator is made up of more than the number of
            // samples we though
            TRACE("[HILT] lost updates!\n");
            lostUpdate = false;
        }

        waitActuatorData();
        hilSerial.write(&actuatorData, sizeof(ActuatorData));
    }
}

/**
 * @brief Waits for the control algorithms to update actuatorData.
 */
void HILTransceiver::waitActuatorData()
{
    miosix::Lock<miosix::FastMutex> l(mutex);
    while (!updated)
    {
        condVar.wait(l);
    }
    updated = false;
}
