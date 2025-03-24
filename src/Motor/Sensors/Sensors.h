/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <Motor/BoardScheduler.h>
#include <Motor/Buses.h>
#include <Motor/Sensors/KuliteThermocouple.h>
#include <drivers/adc/InternalADC.h>
#include <scheduler/TaskScheduler.h>
#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <sensors/SensorManager.h>
#include <sensors/analog/TrafagPressureSensor.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <memory>
#include <vector>

namespace Motor
{

class Sensors : public Boardcore::InjectableWithDeps<Buses, BoardScheduler>
{
public:
    Sensors() {}

    [[nodiscard]] bool start();

    Boardcore::InternalADCData getInternalADCLastSample();
    Boardcore::ADS131M08Data getADS131M08LastSample();
    Boardcore::LPS22DFData getLPS22DFLastSample();
    Boardcore::H3LIS331DLData getH3LIS331DLLastSample();
    Boardcore::LIS2MDLData getLIS2MDLLastSample();
    Boardcore::LSM6DSRXData getLSM6DSRX0LastSample();
    Boardcore::LSM6DSRXData getLSM6DSRX1LastSample();

    Boardcore::PressureData getOxTopTankPressLastSample();
    Boardcore::PressureData getOxBottomTankPress0LastSample();
    Boardcore::PressureData getOxBottomTankPress1LastSample();
    Boardcore::PressureData getN2TankPressLastSample();
    Boardcore::PressureData getCCPressLastSample();
    Boardcore::TemperatureData getTankTempLastSample();
    Boardcore::VoltageData getBatteryVoltageLastSample();

    std::vector<Boardcore::SensorInfo> getSensorInfos();

protected:
    virtual bool postSensorCreationHook() { return true; }

    Boardcore::TaskScheduler& getSensorsScheduler();

    // Digital sensors
    std::unique_ptr<Boardcore::LPS22DF> lps22df;
    std::unique_ptr<Boardcore::H3LIS331DL> h3lis331dl;
    std::unique_ptr<Boardcore::LIS2MDL> lis2mdl;
    std::unique_ptr<Boardcore::LSM6DSRX> lsm6dsrx0;
    std::unique_ptr<Boardcore::LSM6DSRX> lsm6dsrx1;
    std::unique_ptr<Boardcore::ADS131M08> ads131m08;
    std::unique_ptr<Boardcore::InternalADC> internalAdc;

    // Analog sensors
    std::unique_ptr<Boardcore::TrafagPressureSensor> oxTopTankPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> oxBottomTankPressure0;
    std::unique_ptr<Boardcore::TrafagPressureSensor> oxBottomTankPressure1;
    std::unique_ptr<Boardcore::TrafagPressureSensor> n2TankPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> ccPressure;
    std::unique_ptr<KuliteThermocouple> tankTemp;

    std::unique_ptr<Boardcore::SensorManager> manager;

private:
    void lps22dfInit();
    void lps22dfCallback();

    void h3lis331dlInit();
    void h3lis331dlCallback();

    void lis2mdlInit();
    void lis2mdlCallback();

    /// @brief Initialize both lsm6dsrx sensors.
    void lsm6dsrxInit();
    void lsm6dsrx0Callback();
    void lsm6dsrx1Callback();

    void ads131m08Init();
    void ads131m08Callback();

    void internalAdcInit();
    void internalAdcCallback();

    void oxTopTankPressureInit();
    void oxTopTankPressureCallback();

    /// @brief Initialize both ox bottom pressure sensors.
    void oxBottomTankPressureInit();
    void oxBottomTankPressure0Callback();
    void oxBottomTankPressure1Callback();

    void n2TankPressureInit();
    void n2TankPressureCallback();

    void ccPressureInit();
    void ccPressureCallback();

    void tankTempInit();
    void tankTempCallback();

    bool sensorManagerInit();

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");
};

}  // namespace Motor
