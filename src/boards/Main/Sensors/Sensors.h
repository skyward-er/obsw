/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Conterio, Alberto Nidasio
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

#include <diagnostic/PrintLogger.h>
#include <drivers/adc/InternalADC.h>
#include <sensors/ADS131M04/ADS131M04.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/BMX160/BMX160WithCorrection.h>
#include <sensors/MPU9250/MPU9250.h>
#include <sensors/MS5803/MS5803.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSpi.h>
#include <sensors/VN100/VN100.h>
#include <sensors/analog/AnalogLoadCell.h>
#include <sensors/analog/BatteryVoltageSensor.h>
#include <sensors/analog/pressure/honeywell/SSCDRRN015PDA.h>
#include <sensors/analog/pressure/nxp/MPXH6115A.h>
#include <sensors/analog/pressure/nxp/MPXH6400A.h>
#include <utils/Stats/Stats.h>
#ifdef HILSimulation
#include <HIL_algorithms/HILMockKalman.h>
#include <HIL_sensors/HILSensors.h>
#endif  // HILSimulation

namespace Main
{

class Sensors : public Boardcore::Singleton<Sensors>
{
    friend class Boardcore::Singleton<Sensors>;

public:
    bool start();

    bool isStarted();

#ifdef HILSimulation
public:
    /**
     * structure that contains all the sensors used in the simulation
     */
    struct StateComplete
    {
        HILAccelerometer *accelerometer;
        HILBarometer *barometer;
        HILGps *gps;
        HILGyroscope *gyro;
        HILMagnetometer *magnetometer;
        HILTemperature *temperature;
        HILImu *imu;
        HILKalman *kalman;
    } state;
#endif  // HILSimulation

    Boardcore::BMX160 *bmx160 = nullptr;

    Boardcore::BMX160Data getBMX160LastSample();
    Boardcore::BMX160WithCorrectionData getBMX160WithCorrectionLastSample();
    Boardcore::MPU9250Data getMPU9250LastSample();
    Boardcore::MS5803Data getMS5803LastSample();
    Boardcore::UBXGPSData getUbxGpsLastSample();

    Boardcore::ADS131M04Data getADS131M04LastSample();
    Boardcore::MPXH6115AData getStaticPressureLastSample();
    Boardcore::SSCDRRN015PDAData getDifferentialPressureLastSample();
    Boardcore::MPXH6400AData getDplPressureLastSample();
    Boardcore::AnalogLoadCellData getLoadCellLastSample();
    Boardcore::BatteryVoltageSensorData getBatteryVoltageLastSample();

    Boardcore::InternalADCData getInternalADCLastSample();

    bool isCutterPresent();

    /**
     * @brief Blocking function that calibrates the sensors.
     *
     * The calibration works by capturing the mean of the sensor readings and
     * then removing the offsets from the desired values.
     */
    void calibrate();

    std::map<string, bool> getSensorsState();

private:
    Sensors();

    ~Sensors();

    void bmx160Init();
    void bmx160Callback();

    void bmx160WithCorrectionInit();

    void mpu9250Init();

    void ms5803Init();

    void ubxGpsInit();

    void vn100Init();

    void ads131m04Init();

    void staticPressureInit();

    void dplPressureInit();

    void loadCellInit();

    void batteryVoltageInit();

    void internalAdcInit();

    Boardcore::BMX160WithCorrection *bmx160WithCorrection = nullptr;
    Boardcore::MPU9250 *mpu9250                           = nullptr;
    Boardcore::MS5803 *ms5803                             = nullptr;
    Boardcore::UBXGPSSpi *ubxGps                          = nullptr;
    Boardcore::VN100 *vn100                               = nullptr;

    Boardcore::ADS131M04 *ads131m04                 = nullptr;
    Boardcore::MPXH6115A *staticPressure            = nullptr;
    Boardcore::MPXH6400A *dplPressure               = nullptr;
    Boardcore::AnalogLoadCell *loadCell             = nullptr;
    Boardcore::BatteryVoltageSensor *batteryVoltage = nullptr;

    Boardcore::InternalADC *internalAdc = nullptr;

    Boardcore::SensorManager *sensorManager = nullptr;

    Boardcore::SensorManager::SensorMap_t sensorsMap;

    bool calibrating = false;
    Boardcore::Stats ms5803Stats;
    Boardcore::Stats staticPressureStats;
    Boardcore::Stats dplPressureStats;
    Boardcore::Stats loadCellStats;

    float cutterSensingMean = 0;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");
};

}  // namespace Main
