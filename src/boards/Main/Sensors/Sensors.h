/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio, Alberto Nidasio
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
#include <sensors/ADS1118/ADS1118.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/LIS3MDL/LIS3MDL.h>
#include <sensors/MS5803/MS5803.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSerial.h>
#include <sensors/analog/BatteryVoltageSensor.h>
#include <sensors/analog/pressure/MPXHZ6130A/MPXHZ6130A.h>
#include <sensors/analog/pressure/honeywell/SSCDANN030PAA.h>
#include <sensors/analog/pressure/honeywell/SSCDRRN015PDA.h>

namespace Main
{

class Sensors : public Boardcore::Singleton<Sensors>
{
    friend class Boardcore::Singleton<Sensors>;

public:
    Boardcore::BMX160 *bmx160       = nullptr;
    Boardcore::LIS3MDL *lis3mdl     = nullptr;
    Boardcore::MS5803 *ms5803       = nullptr;
    Boardcore::UBXGPSSerial *ubxGps = nullptr;

    Boardcore::ADS1118 *ads1118             = nullptr;
    Boardcore::MPXHZ6130A *staticPressure   = nullptr;
    Boardcore::SSCDANN030PAA *dplPressure   = nullptr;
    Boardcore::SSCDRRN015PDA *pitotPressure = nullptr;

    Boardcore::InternalADC *internalAdc             = nullptr;
    Boardcore::BatteryVoltageSensor *batteryVoltage = nullptr;

    bool start();

private:
    Sensors();

    ~Sensors();

    void bmx160Init();
    void bmx160Callback();

    void lis3mdlInit();

    void ms5803Init();

    void ubxGpsInit();
    void ubxGpsCallback();

    void ads1118Init();

    void staticPressureInit();

    void dplPressureInit();

    void pitotPressureInit();
    void pitotPressureCallback();

    void internalAdcInit();

    void batteryVoltageInit();

    Boardcore::SensorManager *sensorManager = nullptr;

    Boardcore::SensorManager::SensorMap_t sensorsMap;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");
};

}  // namespace Main
