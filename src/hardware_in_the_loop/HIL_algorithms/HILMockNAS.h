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

#include "NavigationSystem/NASController.h"
#include "hardware_in_the_loop/HIL_sensors/HILAccelerometer.h"
#include "hardware_in_the_loop/HIL_sensors/HILBarometer.h"
#include "hardware_in_the_loop/HIL_sensors/HILGps.h"
#include "hardware_in_the_loop/HIL_sensors/HILGyroscope.h"
#include "hardware_in_the_loop/HIL_sensors/HILMagnetometer.h"
#include "hardware_in_the_loop/HIL_sensors/HILSensor.h"

struct HILNasData : public TimestampData
{
    HILNasData() : TimestampData{0} {}

    HILNasData(uint64_t t) : TimestampData{t} {}

    float z;
    float vz;
    float vMod;
};

class HILNas : public HILSensor<HILNasData>
{
public:
    HILNas(HILTransceiver *matlab, int n_data_kalm,
           NASController<HILImuData, HILBaroData, HILGpsData> *nas)
        : HILSensor(matlab, n_data_kalm)
    {
        this->nas = nas->getNAS();
    }

protected:
    /**
     * @brief Updates the internal structure of the fake sensor from the
     * structure received from the simulator.
     *
     * Takes the next unread sample available, continues sending the last sample
     * with the old timestamp if we already read all the samples.
     */
    HILNasData updateData() override
    {
        HILNasData tempData;
        NASData nas_data   = nas->getNasData();
        tempData.z         = nas_data.pz;
        tempData.vz        = nas_data.vz;
        tempData.vMod      = nas_data.vmod;
        tempData.timestamp = nas_data.timestamp;

        return tempData;
    }

    NAS<HILImuData, HILBaroData, HILGpsData> *nas;
};
