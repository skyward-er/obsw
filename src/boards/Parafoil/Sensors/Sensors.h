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

#include <drivers/adc/InternalADC.h>
#include <drivers/adc/InternalTemp.h>
#include <scheduler/TaskScheduler.h>
#include <sensors/ADS1118/ADS1118.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/BMX160/BMX160WithCorrection.h>
#include <sensors/LIS3MDL/LIS3MDL.h>
#include <sensors/MS5803/MS5803.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSerial.h>
#include <sensors/analog/BatteryVoltageSensor.h>
#include <sensors/analog/Pitot/Pitot.h>
#include <sensors/analog/pressure/honeywell/SSCDANN030PAA.h>
#include <sensors/analog/pressure/honeywell/SSCDRRN015PDA.h>
#include <sensors/analog/pressure/nxp/MPXHZ6130A.h>
#ifdef HILSimulation
#include <HIL_algorithms/HILMockKalman.h>
#include <HIL_sensors/HILSensors.h>
#endif  // HILSimulation
#include <Parafoil/ModuleHelper/ParafoilModule.h>

namespace Parafoil
{

class Sensors : public ParafoilModule
{

public:
    Sensors();

    ~Sensors();

    bool startModule() override;

    bool isStarted();

#ifdef HILSimulation
public:
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
#endif  // HILSimulation

    Boardcore::BMX160* bmx160;

    Boardcore::BMX160Data getBMX160LastSample();
    Boardcore::BMX160WithCorrectionData getBMX160WithCorrectionLastSample();
    Boardcore::LIS3MDLData getMagnetometerLIS3MDLLastSample();
    Boardcore::MS5803Data getMS5803LastSample();
    Boardcore::UBXGPSData getUbxGpsLastSample();

    Boardcore::ADS1118Data getADS1118LastSample();
    Boardcore::MPXHZ6130AData getStaticPressureLastSample();
    Boardcore::SSCDANN030PAAData getDplPressureLastSample();
    Boardcore::SSCDRRN015PDAData getPitotPressureLastSample();
    Boardcore::PitotData getPitotLastSample();
    Boardcore::InternalADCData getInternalADCLastSample();
    Boardcore::BatteryVoltageSensorData getBatteryVoltageLastSample();

    /**
     * @brief Blocking function that calibrates the sensors.
     *
     * The calibration works by capturing the mean of the sensor readings and
     * then removing the offsets from the desired values.
     */
    void calibrate();

    void pitotSetReferenceAltitude(float altitude);
    void pitotSetReferenceTemperature(float temperature);

    std::map<string, bool> getSensorsState();

private:
    void bmx160Init();
    void bmx160Callback();

    void bmx160WithCorrectionInit();

    void lis3mdlInit();

    void ms5803Init();

    void ubxGpsInit();

    void ads1118Init();

    void staticPressureInit();

    void dplPressureInit();

    void pitotPressureInit();
    void pitotInit();

    void internalADCInit();
    void batteryVoltageInit();

    void internalTempInit();

    Boardcore::BMX160WithCorrection* bmx160WithCorrection;
    Boardcore::LIS3MDL* lis3mdl;
    Boardcore::MS5803* ms5803;  // barometro digitale
    Boardcore::UBXGPSSerial* ubxGps;

    Boardcore::ADS1118* ads1118;  // adc
    Boardcore::MPXHZ6130A* staticPressure;
    Boardcore::SSCDANN030PAA* dplPressure;
    Boardcore::SSCDRRN015PDA* pitotPressure;

    Boardcore::Pitot* pitot;
    Boardcore::InternalADC* internalADC;
    Boardcore::BatteryVoltageSensor* batteryVoltage;
    Boardcore::InternalTemp* internalTemp = nullptr;

    Boardcore::SensorManager* sensorManager = nullptr;

    Boardcore::SensorManager::SensorMap_t sensorsMap;

    bool calibrating = false;
    Boardcore::Stats ms5803Stats;
    Boardcore::Stats staticPressureStats;
    Boardcore::Stats dplPressureStats;
    Boardcore::Stats pitotPressureStats;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");
};

}  // namespace Parafoil
