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

#include <Motor/Buses.h>
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

    explicit Sensors(Boardcore::TaskScheduler* sched, Motor::Buses* buses);

    ~Sensors();

    virtual bool start();

    virtual void calibrate();

protected:
   /**
     * @brief Method to put a sensor in the sensorMap with the relative infos
     */
    template <typename T>
    void registerSensor(Boardcore::Sensor<T>* sensor, const std::string& name,
                        uint32_t period, std::function<void(void)> callback)
    {
        // Emplace the sensor inside the map
        Boardcore::SensorInfo info(name, period, callback);
        sensorsMap.emplace(std::make_pair(sensor, info));
    }

    // Creation and callbacks methods
    void adcCreation();
    virtual void adcCallback();

    void batteryCreation();
    virtual void batteryCallback();

    void h3lis331dlCreation();
    virtual void h3lis331dlCallback();

    void lps22dfCreation();
    virtual void lps22dfCallback();

    void max31856Creation();
    virtual void max31856Callback();

    void ads131m08Creation();
    virtual void ads131m08Callback();

    void chamberPressureCreation();
    virtual void chamberPressureCallback();

    void tankPressure1Creation();
    virtual void tankPressure1Callback();

    void tankPressure2Creation();
    virtual void tankPressure2Callback();

    void servosCurrentCreation();
    virtual void servosCurrentCallback();

    Boardcore::SensorManager::SensorMap_t sensorsMap;
    Boardcore::SensorManager* manager = nullptr;
    Boardcore::TaskScheduler* scheduler     = nullptr;
    Motor::Buses* buses                  = nullptr;

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