/* Copyright (c) 2020 Skyward Experimental Rocketry
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

#pragma once

#include <typeinfo>

#include "HILSensorsData.h"
#include "HILTimestampManagement.h"
#include "TimestampTimer.h"
#include "hardware_in_the_loop/HILConfig.h"
#include "hardware_in_the_loop/simulator_communication/HILTransceiver.h"
#include "math/Vec3.h"
#include "sensors/Sensor.h"
#include "sensors/SensorData.h"

/**
 * @brief Fake sensor base used for the simulation. Every sensor for the
 * simulation should extend this class.
 *
 * This class is used to simulate as near as possible the situation of the
 * OBSW during the flight, using fake sensors classes instead of the real
 * ones, taking their data from the data received from a simulator.
 */
template <typename HILSensorData>
class HILSensor : public virtual HILTimestampManagement,
                  public virtual Sensor<HILSensorData>
{
public:
    /**
     * @brief constructor of the fake sensor used for the simulation.
     *
     * @param matlab reference of the MatlabTransceiver object that deals with
     * the simulator
     * @param n_data_sensor number of samples in every period of simulation
     */
    HILSensor(HILTransceiver *matlab, int n_data_sensor)
    {
        this->sensorData    = matlab->getSensorData();
        this->n_data_sensor = n_data_sensor;

        /* Registers the sensor on the MatlabTransceiver to be notified when a
         * new packet of simulated data arrives */
        matlab->addResetSampleCounter(this);
    }

    /**
     * @brief sets the sample counter to 0.
     *
     * Updates the reference timestamp, resets the sampleCounter and clears the
     * last_error variable. Called by the HILTransceiver when receives a new
     * simulated period.
     */
    void resetSampleCounter() override
    {
        this->last_error = SensorErrors::NO_ERRORS;
        sampleCounter    = 0;
    }

    /**
     * @brief Initializes the fake sensor.
     */
    bool init() override
    {
        if (initialized)
        {
            this->last_error = SensorErrors::ALREADY_INIT;
            TRACE("ALREADY INITIALIZED!");
        }
        else
        {
            initialized = true;
        }

        return initialized;
    }

    bool selfTest() override { return true; }

protected:
    /**
     * @brief Updates the internal structure of the fake sensor from the
     * structure received from the simulator.
     *
     * Takes the next unread sample available, continues sending the last sample
     * with the old timestamp if we already read all the samples.
     */
    HILSensorData sampleImpl() override
    {
        if (initialized)
        {
            /* updates the last_sensor only if there is still data to be read */
            if (sampleCounter >= n_data_sensor)
            {
                this->last_error = SensorErrors::NO_NEW_DATA;
                /*TRACE("[%s] NO NEW DATA! Simulation error\n",
                      typeid(this).name());*/
            }
            else if (this->last_error != SensorErrors::NO_NEW_DATA)
            {
                return updateData();
            }
        }
        else
        {
            this->last_error = SensorErrors::NOT_INIT;
            TRACE(
                "[HILSensor] sampleImpl() : not initialized, unable to "
                "sample data \n");
        }

        return this->last_sample;
    }

    /**
     * @brief updates the timestamp and increments the sampleCounter.
     * WARNING: You should call this method after all the values has been
     * updated, it modifies the sampleCounter!
     * @return the timestamp of the sample
     */
    uint64_t updateTimestamp()
    {
        sampleCounter++;
        return Boardcore::TimestampTimer::getInstance().getTimestamp();
    }

    /**
     * @brief Function that updates the sensor structure with new data.
     *
     * Sensor struct updated from MatlabTransceiver::sensorData.
     * WARNING: This method should call **AT THE END** the updateTimestamp
     * method.
     */
    virtual HILSensorData updateData() = 0;

    bool initialized  = false;
    int sampleCounter = 0;     /**< counter of the next sample to take */
    int n_data_sensor;         /**< number of samples in every period */
    SimulatorData *sensorData; /**< reference to the SensorData structure */
};
