/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <Motor/Sensors/ChamberPressureSensor/ChamberPressureSensor.h>
#include <Motor/Sensors/TankPressureSensor1/TankPressureSensor1.h>
#include <Motor/Sensors/TankPressureSensor2/TankPressureSensor2.h>
#include <drivers/adc/InternalADC.h>
#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/MAX31856/MAX31856.h>
#include <sensors/SensorManager.h>
#include <sensors/analog/BatteryVoltageSensor.h>
#include <sensors/analog/CurrentSensor.h>
#include <sensors/analog/pressure/AnalogPressureSensor.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Motor
{

class Sensors : public Boardcore::Module
{
public:
    virtual Boardcore::InternalADCData getADCData();
    virtual Boardcore::BatteryVoltageSensorData getBatteryData();
    virtual Boardcore::H3LIS331DLData getH3LIS331DLData();
    virtual Boardcore::LIS2MDLData getLIS2MDLData();
    virtual Boardcore::LPS22DFData getLPS22DFData();
    virtual Boardcore::ADS131M08Data getADS131M08Data();
    virtual Boardcore::MAX31856Data getMAX31856Data();
    virtual Boardcore::ChamberPressureSensorData getChamberPressureSensorData();
    virtual Boardcore::TankPressureSensor1Data getTankPressureSensor1Data();
    virtual Boardcore::TankPressureSensor2Data getTankPressureSensor2Data();
    virtual Boardcore::CurrentData getServoCurrentData();

    explicit Sensors(Boardcore::TaskScheduler* sched);

    ~Sensors();

    virtual bool start();

    virtual void calibrate();

protected:
    virtual void adcInit();
    virtual void adcCallback();

    virtual void batteryInit();
    virtual void batteryCallback();

    virtual void h3lis331dlInit();
    virtual void h3lis331dlCallback();

    virtual void lis2mdlInit();
    virtual void lis2mdlCallback();

    virtual void lps22dfInit();
    virtual void lps22dfCallback();

    virtual void max31856Init();
    virtual void max31856Callback();

    virtual void ads131m08Init();
    virtual void ads131m08Callback();

    virtual void chamberPressureInit();
    virtual void chamberPressureCallback();

    virtual void tankPressure1Init();
    virtual void tankPressure1Callback();

    virtual void tankPressure2Init();
    virtual void tankPressure2Callback();

    virtual void servosCurrentInit();
    virtual void servosCurrentCallback();

    Boardcore::SensorManager::SensorMap_t sensorsMap;
    Boardcore::SensorManager* sensorManager = nullptr;
    Boardcore::TaskScheduler* scheduler     = nullptr;

private:
    Boardcore::InternalADC* adc                       = nullptr;
    Boardcore::BatteryVoltageSensor* battery          = nullptr;
    Boardcore::H3LIS331DL* h3lis331dl                 = nullptr;
    Boardcore::LIS2MDL* lis2mdl                       = nullptr;
    Boardcore::LPS22DF* lps22df                       = nullptr;
    Boardcore::MAX31856* max31856                     = nullptr;
    Boardcore::ADS131M08* ads131m08                   = nullptr;
    Boardcore::ChamberPressureSensor* chamberPressure = nullptr;
    Boardcore::TankPressureSensor1* tankPressure1     = nullptr;
    Boardcore::TankPressureSensor2* tankPressure2     = nullptr;
    Boardcore::CurrentSensor* servosCurrent           = nullptr;
};

}  // namespace Motor