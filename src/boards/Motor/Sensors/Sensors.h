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
#include <sensors/LSM6DSRX/LSM6DSRX.h>
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
    Boardcore::InternalADC* adc                       = nullptr;
    Boardcore::BatteryVoltageSensor* battery          = nullptr;
    Boardcore::LSM6DSRX* lsm6                         = nullptr;
    Boardcore::H3LIS331DL* h3lis                      = nullptr;
    Boardcore::LIS2MDL* lis2                          = nullptr;
    Boardcore::LPS22DF* lps22                         = nullptr;
    Boardcore::ADS131M08* ads131                      = nullptr;
    Boardcore::MAX31856* max                          = nullptr;
    Boardcore::ChamberPressureSensor* chamberPressure = nullptr;
    Boardcore::TankPressureSensor1* tankPressure1     = nullptr;
    Boardcore::TankPressureSensor2* tankPressure2     = nullptr;
    Boardcore::CurrentSensor* servosCurrent           = nullptr;

    Boardcore::SensorManager::SensorMap_t sensorsMap;
    Boardcore::SensorManager* sensorManager = nullptr;

    Sensors();

    ~Sensors();

    bool start();

    void calibrate();

private:
    void adcInit();

    void batteryInit();

    void lsm6Init();

    void h3lisInit();

    void lis2Init();

    void lps22Init();

    void maxInit();

    void ads131Init();

    void chamberPressureInit();

    void tankPressure1Init();

    void tankPressure2Init();

    void servosCurrentInit();
};

}  // namespace Motor