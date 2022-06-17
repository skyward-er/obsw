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
#include <sensors/analog/BatteryVoltageSensor.h>
#include <sensors/analog/pressure/nxp/MPXH6115A.h>
#include <sensors/analog/pressure/nxp/MPXH6400A.h>

namespace Main
{

class Sensors : public Boardcore::Singleton<Sensors>
{
    friend class Boardcore::Singleton<Sensors>;

public:
    Boardcore::BMX160 *bmx160                             = nullptr;
    Boardcore::BMX160WithCorrection *bmx160WithCorrection = nullptr;
    Boardcore::MPU9250 *mpu9250                           = nullptr;
    Boardcore::MS5803 *ms5803 = nullptr;  // digital pressure sensor

    Boardcore::ADS131M04 *ads131m04      = nullptr;
    Boardcore::MPXH6115A *staticPressure = nullptr;
    Boardcore::MPXH6400A *dplPressure    = nullptr;

    Boardcore::InternalADC *internalAdc             = nullptr;
    Boardcore::BatteryVoltageSensor *batteryVoltage = nullptr;

    bool start();

private:
    Sensors();

    ~Sensors();

    void bmx160Init();
    void bmx160Callback();

    void bmx160WithCorrectionInit();

    void mpu9250Init();

    void ms5803Init();

    void ubxGpsInit();
    void ubxGpsCallback();

    void ads131m04Init();

    void staticPressureInit();

    void dplPressureInit();

    void internalAdcInit();

    void batteryVoltageInit();

    Boardcore::SensorManager *sensorManager = nullptr;

    Boardcore::SensorManager::SensorMap_t sensorsMap;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");
};

}  // namespace Main
