/* Copyright (c) 2020-2021 Skyward Experimental Rocketry
 * Authors: Emilio Corigliano
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include "ActiveObject.h"
#include "DeathStack/Algorithm.h"
#include "SerialInterface.h"
#include "TimestampTimer.h"
#include "hardware_in_the_loop/HILFlightPhasesManager.h"
#include "hardware_in_the_loop/HIL_sensors/HILTimestampManagement.h"
#include "math/Vec3.h"

using namespace miosix;

/**
 * @brief HILTranceiver is a Singleton and provides an easy interface for the
 * control algorithms to send and receive data during a simulation
 */
class HILTransceiver : public ActiveObject
{
public:
    /**
     * @brief Construct a serial connection attached to a control algorithm
     */
    HILTransceiver(HILFlightPhasesManager *fpMgr)
        : flightPhasesManager(fpMgr), actuatorData(0.0f)
    {
        serial = new SerialInterface(SIM_BAUDRATE, SIM_SERIAL_PORT_NUM);

        // initializing the serial connection
        if (!serial->init())
        {
            TRACE("[HIL] Wrong initialization\n");
        }
    }

    /**
     * @brief sets the actuator data and then wakes up the MatlabTransceiver
     * thread in order to send the data back to the simulator (called by the
     * control algorithm)
     * @param actuatorData sets the data that will be sent to the simulator
     */
    void setActuatorData(ActuatorData actuatorData)
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
    SimulatorData *getSensorData() { return &sensorData; }

    /**
     * @brief adds to the resetSampleCounter list an object that has to be
     * notified when a new packet of data is arrived from the simulator
     *
     * @param t SimTimestampManagement object
     */
    void addResetSampleCounter(HILTimestampManagement *t)
    {
        sensorsTimestamp.push_back(t);
    }

    /**
     * @brief adds to the notifyToBegin list an object that has to be started
     * when the first packet from the simulator arrives.
     *
     * @param algorithm Algorithm object to be started
     */
    /*void setIsAerobrakePhase(bool isAerobrakePhase)
    {
        this->isAerobrakePhase = isAerobrakePhase;
        if (!isAerobrakePhase)
        {
            actuatorData = 0;
        }
    }*/

private:
    /**
     * @brief the thread deals with the communication between the simulator and
     * the mock sensors
     *
     * The first read is done in the init() function;
     *
     * After the first time the data is received the loop of this thread is:
     * - reads the simulated data and copies them in the SensorData structure
     * - notifies every sensor that new data arrived,
     * - waits for the control algorithms to update the actuator data
     * - sends back the value to the simulator
     */
    void run() override
    {
        TRACE("[HIL] Transceiver started\n");

        while (true)
        {
            /* Scope of the temporary data read from serial and pausing the
             * kernel in order to copy the data in the shared structure */
            {
                SimulatorData tempData;
                serial->recvData<SimulatorData>(&tempData);
                PauseKernelLock kLock;
                memmove(&sensorData, &tempData, sizeof(SimulatorData));
            }

            if (!receivedFirstPacket)
            {
                receivedFirstPacket = true;
                flightPhasesManager->setFlagFlightPhase(SIMULATION_STARTED,
                                                        true);
            }

            /*
             * notify all sensors that a new set of data is arrived
             * [REVIEW] Could be moved in HILFlightPhasesManager
             */
            for (auto st : sensorsTimestamp)
            {
                st->resetSampleCounter();
            }

            // trigger events relative to the flight phases
            flightPhasesManager->processFlags(sensorData.flags);

            waitActuatorData();
            serial->sendData<ActuatorData>(&actuatorData);
        }
    }

    /**
     * @brief Waits for the control algorithm(s) to update actuatorData.
     */
    void waitActuatorData()
    {
        Lock<FastMutex> l(mutex);
        while (!updated)
        {
            condVar.wait(l);
        }
        updated = false;
    }

    SerialInterface *serial;
    // bool isAerobrakePhase    = false;
    bool receivedFirstPacket = false;
    bool updated             = false;
    HILFlightPhasesManager *flightPhasesManager;
    SimulatorData sensorData;
    ActuatorData actuatorData;
    std::vector<HILTimestampManagement *> sensorsTimestamp;
    FastMutex mutex;
    ConditionVariable condVar;
};