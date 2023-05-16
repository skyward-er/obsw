/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include "HIL.h"
#include "HIL_algorithms/HILMockKalman.h"
#include "HIL_sensors/HILSensors.h"
#include "Sensors.h"

namespace Parafoil
{

class HILSensors : public Sensors
{

public:
    HILSensors();

    ~HILSensors();

    bool startModule() override;

    bool isStarted() override;

    Boardcore::BMX160Data getBMX160LastSample() override;
    Boardcore::BMX160WithCorrectionData getBMX160WithCorrectionLastSample()
        override;
    Boardcore::LIS3MDLData getMagnetometerLIS3MDLLastSample() override;
    Boardcore::MS5803Data getMS5803LastSample() override;
    Boardcore::UBXGPSData getUbxGpsLastSample() override;

    Boardcore::ADS1118Data getADS1118LastSample() override;
    Boardcore::MPXHZ6130AData getStaticPressureLastSample() override;
    Boardcore::SSCDANN030PAAData getDplPressureLastSample() override;
    Boardcore::SSCDRRN015PDAData getPitotPressureLastSample() override;
    Boardcore::PitotData getPitotLastSample() override;
    Boardcore::InternalADCData getInternalADCLastSample() override;
    Boardcore::BatteryVoltageSensorData getBatteryVoltageLastSample() override;

    /**
     * @brief Blocking function that calibrates the sensors.
     *
     * The calibration works by capturing the mean of the sensor readings and
     * then removing the offsets from the desired values.
     */
    void calibrate() override;

private:
    /**
     * structure that contains all the sensors used in the simulation
     */
    struct StateComplete
    {
        HILAccelerometer* accelerometer;
        HILBarometer* barometer;
        HILPitot* pitot;
        HILGps* gps;
        HILGyroscope* gyro;
        HILMagnetometer* magnetometer;
        HILTemperature* temperature;
        HILImu* imu;
        HILKalman* kalman;
    } state;
};

}  // namespace Parafoil
