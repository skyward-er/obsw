/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Luca Conterio, Matteo Pignataro
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

#include "Sensors.h"

#include <Motor/Buses.h>
#include <Motor/Configs/SensorsConfig.h>
#include <drivers/interrupt/external_interrupts.h>
#include <interfaces-impl/hwmapping.h>
#include <logger/Logger.h>

using namespace std;
using namespace miosix;
using namespace Boardcore;
using namespace Motor::SensorsConfig;

namespace Motor
{

InternalADCData Sensors::getADCData()
{
    miosix::PauseKernelLock lock;
    return adc != nullptr ? adc->getLastSample() : InternalADCData{};
}

BatteryVoltageSensorData Sensors::getBatteryData()
{
    miosix::PauseKernelLock lock;
    return battery != nullptr ? battery->getLastSample()
                              : BatteryVoltageSensorData{};
}

// LSM6DSRXData Sensors::getLSM6DSRXData()
// {
//     miosix::PauseKernelLock lock;
//     return lsm6dsrx != nullptr ? lsm6dsrx->getLastSample() : LSM6DSRXData{};
// }

H3LIS331DLData Sensors::getH3LIS331DLData()
{
    miosix::PauseKernelLock lock;
    return h3lis331dl != nullptr ? h3lis331dl->getLastSample()
                                 : H3LIS331DLData{};
}

LIS2MDLData Sensors::getLIS2MDLData()
{
    miosix::PauseKernelLock lock;
    return lis2mdl != nullptr ? lis2mdl->getLastSample() : LIS2MDLData{};
}

LPS22DFData Sensors::getLPS22DFData()
{
    miosix::PauseKernelLock lock;
    return lps22df != nullptr ? lps22df->getLastSample() : LPS22DFData{};
}

ADS131M08Data Sensors::getADS131M08Data()
{
    miosix::PauseKernelLock lock;
    return ads131m08 != nullptr ? ads131m08->getLastSample() : ADS131M08Data{};
}

MAX31856Data Sensors::getMAX31856Data()
{
    miosix::PauseKernelLock lock;
    return max31856 != nullptr ? max31856->getLastSample() : MAX31856Data{};
}

ChamberPressureSensorData Sensors::getChamberPressureSensorData()
{
    miosix::PauseKernelLock lock;
    return chamberPressure != nullptr ? chamberPressure->getLastSample()
                                      : ChamberPressureSensorData{};
}

TankPressureSensor1Data Sensors::getTankPressureSensor1Data()
{
    miosix::PauseKernelLock lock;
    return tankPressure1 != nullptr ? tankPressure1->getLastSample()
                                    : TankPressureSensor1Data{};
}

TankPressureSensor2Data Sensors::getTankPressureSensor2Data()
{
    miosix::PauseKernelLock lock;
    return tankPressure2 != nullptr ? tankPressure2->getLastSample()
                                    : TankPressureSensor2Data{};
}

CurrentData Sensors::getServoCurrentData()
{
    miosix::PauseKernelLock lock;
    return servosCurrent != nullptr ? servosCurrent->getLastSample()
                                    : CurrentData{};
}

Sensors::Sensors(TaskScheduler* sched) : scheduler(sched) {}

Sensors::~Sensors() {}

bool Sensors::start()
{
    adcInit();
    batteryInit();
    // lsm6dsrxInit();
    h3lis331dlInit();
    // lis2mdlInit();
    lps22dfInit();
    max31856Init();
    ads131m08Init();
    chamberPressureInit();
    tankPressure1Init();
    tankPressure2Init();
    servosCurrentInit();

    sensorManager = new SensorManager(sensorsMap, scheduler);

    return sensorManager->start();
}

void Sensors::calibrate()
{
    if (ads131m08 != nullptr)
    {
        ads131m08->calibrateOffset(ADS131_SERVO_CURRENT_CH);
    }
}

void Sensors::adcInit()
{
    adc = new InternalADC(ADC1);

    adc->enableTemperature();
    adc->enableVbat();
    adc->enableChannel(ADC_BATTERY_VOLTAGE_CH);

    SensorInfo info("ADC", SAMPLE_PERIOD_ADC,
                    bind(&Sensors::adcCallback, this));

    sensorsMap.emplace(make_pair(adc, info));
}

void Sensors::batteryInit()
{
    function<ADCData()> getADCVoltage(
        bind(&InternalADC::getVoltage, adc, ADC_BATTERY_VOLTAGE_CH));

    battery =
        new BatteryVoltageSensor(getADCVoltage, ADC_BATTERY_VOLTAGE_COEFF);

    SensorInfo info("BATTERY", SAMPLE_PERIOD_ADC,
                    bind(&Sensors::batteryCallback, this));

    sensorsMap.emplace(make_pair(battery, info));
}

// void Sensors::lsm6dsrxInit()
// {
//     SPIBus& spi1 = ModuleManager::getInstance().get<Buses>()->spi1;

//     lsm6dsrx = new LSM6DSRX(spi1, peripherals::lsm6dsrx::cs::getPin(),
//                             LSM6_SPI_CONFIG, LSM6_SENSOR_CONFIG);

//     SensorInfo info("LSM6DSRX", SAMPLE_PERIOD_LSM6,
//                     bind(&Sensors::lsm6dsrxCallback, this));

//     sensorsMap.emplace(make_pair(lsm6dsrx, info));
// }

void Sensors::h3lis331dlInit()
{
    SPIBus& spi3 = ModuleManager::getInstance().get<Buses>()->spi3;

    h3lis331dl = new H3LIS331DL(spi3, peripherals::h3lis331dl::cs::getPin(),
                                H3LIS_ODR, H3LIS_BDU, H3LIS_FSR);

    SensorInfo info("H3LIS331DL", SAMPLE_PERIOD_H3LIS,
                    bind(&Sensors::h3lis331dlCallback, this));

    sensorsMap.emplace(make_pair(h3lis331dl, info));
}

void Sensors::lis2mdlInit()
{
    SPIBus& spi3 = ModuleManager::getInstance().get<Buses>()->spi3;

    lis2mdl = new LIS2MDL(spi3, peripherals::lis2mdl::cs::getPin(),
                          LIS2_SPI_CONFIG, LIS2_SENSOR_CONFIG);

    SensorInfo info("LIS2MDL", SAMPLE_PERIOD_LIS2,
                    bind(&Sensors::lis2mdlCallback, this));

    sensorsMap.emplace(make_pair(lis2mdl, info));
}

void Sensors::lps22dfInit()
{
    SPIBus& spi3 = ModuleManager::getInstance().get<Buses>()->spi3;

    lps22df = new LPS22DF(spi3, peripherals::lps22df::cs::getPin(),
                          LPS22_SPI_CONFIG, LPS22_SENSOR_CONFIG);

    SensorInfo info("LPS22", SAMPLE_PERIOD_LPS22,
                    bind(&Sensors::lps22dfCallback, this));

    sensorsMap.emplace(make_pair(lps22df, info));
}

void Sensors::max31856Init()
{
    SPIBus& spi3 = ModuleManager::getInstance().get<Buses>()->spi3;

    max31856 = new MAX31856(spi3, peripherals::max31856::cs::getPin());

    SensorInfo info("MAX31856", SAMPLE_PERIOD_MAX,
                    bind(&Sensors::max31856Callback, this));

    sensorsMap.emplace(make_pair(max31856, info));
}

void Sensors::ads131m08Init()
{
    SPIBus& spi4 = ModuleManager::getInstance().get<Buses>()->spi4;

    ads131m08 = new ADS131M08(spi4, peripherals::ads131m08::cs::getPin(),
                              ADS131_SPI_CONFIG, ADS131_SENSOR_CONFIG);

    SensorInfo info("ADS131M08", SAMPLE_PERIOD_ADS131,
                    bind(&Sensors::ads131m08Callback, this));

    sensorsMap.emplace(make_pair(ads131m08, info));
}

void Sensors::chamberPressureInit()
{
    if (ads131m08 == nullptr)
    {
        return;
    }

    function<ADCData()> getADCVoltage(
        [&](void)
        {
            auto data = ads131m08->getLastSample();
            return data.getVoltage(ADS131_CHAMBER_PRESSURE_CH);
        });

    function<float(float)> voltageToPressure(
        [](float voltage)
        {
            float current =
                voltage / CHAMBER_PRESSURE_SHUNT - CHAMBER_PRESSURE_CURR_MIN;
            return current * CHAMBER_PRESSURE_COEFF;
        });

    chamberPressure =
        new ChamberPressureSensor(getADCVoltage, voltageToPressure);

    SensorInfo info("CHAMBER_PRESSURE", SAMPLE_PERIOD_ADS131,
                    bind(&Sensors::chamberPressureCallback, this));

    sensorsMap.emplace(make_pair(chamberPressure, info));
}

void Sensors::tankPressure1Init()
{
    if (ads131m08 == nullptr)
    {
        return;
    }

    function<ADCData()> getADCVoltage(
        [&](void)
        {
            auto data = ads131m08->getLastSample();
            return data.getVoltage(ADS131_TANK_PRESSURE_1_CH);
        });

    function<float(float)> voltageToPressure(
        [](float voltage)
        {
            float current =
                voltage / TANK_PRESSURE_1_SHUNT - TANK_PRESSURE_1_CURR_MIN;
            return current * TANK_PRESSURE_1_COEFF;
        });

    tankPressure1 = new TankPressureSensor1(getADCVoltage, voltageToPressure);

    SensorInfo info("TANK_PRESSURE_1", SAMPLE_PERIOD_ADS131,
                    bind(&Sensors::tankPressure1Callback, this));

    sensorsMap.emplace(make_pair(tankPressure1, info));
}

void Sensors::tankPressure2Init()
{
    if (ads131m08 == nullptr)
    {
        return;
    }

    function<ADCData()> getADCVoltage(
        [&](void)
        {
            auto data = ads131m08->getLastSample();
            return data.getVoltage(ADS131_TANK_PRESSURE_2_CH);
        });

    function<float(float)> voltageToPressure(
        [](float voltage)
        {
            float current =
                voltage / TANK_PRESSURE_2_SHUNT - TANK_PRESSURE_2_CURR_MIN;
            return current * TANK_PRESSURE_2_COEFF;
        });

    tankPressure2 = new TankPressureSensor2(getADCVoltage, voltageToPressure);

    SensorInfo info("TANK_PRESSURE_2", SAMPLE_PERIOD_ADS131,
                    bind(&Sensors::tankPressure2Callback, this));

    sensorsMap.emplace(make_pair(tankPressure2, info));
}

void Sensors::servosCurrentInit()
{
    if (ads131m08 == nullptr)
    {
        return;
    }

    function<ADCData()> getADCVoltage(
        [&](void)
        {
            auto data = ads131m08->getLastSample();
            return data.getVoltage(ADS131_SERVO_CURRENT_CH);
        });

    function<float(float)> voltageToCurrent(
        [](float voltage) { return voltage * SERVO_CURRENT_COEFF; });

    servosCurrent = new CurrentSensor(getADCVoltage, voltageToCurrent);

    SensorInfo info("SERVOS_CURRENT", SAMPLE_PERIOD_ADS131,
                    bind(&Sensors::servosCurrentCallback, this));

    sensorsMap.emplace(make_pair(servosCurrent, info));
}

void Sensors::adcCallback() { Logger::getInstance().log(adc->getLastSample()); }

void Sensors::batteryCallback()
{
    Logger::getInstance().log(battery->getLastSample());
}

// void Sensors::lsm6dsrxCallback()
// {
//     Logger::getInstance().log(lsm6dsrx->getLastSample());
// }

void Sensors::h3lis331dlCallback()
{
    Logger::getInstance().log(h3lis331dl->getLastSample());
}

void Sensors::lis2mdlCallback()
{
    Logger::getInstance().log(lis2mdl->getLastSample());
}

void Sensors::lps22dfCallback()
{
    Logger::getInstance().log(lps22df->getLastSample());
}

void Sensors::max31856Callback()
{
    Logger::getInstance().log(max31856->getLastSample());
}

void Sensors::ads131m08Callback()
{
    Logger::getInstance().log(ads131m08->getLastSample());
}

void Sensors::chamberPressureCallback()
{
    Logger::getInstance().log(chamberPressure->getLastSample());
}

void Sensors::tankPressure1Callback()
{
    Logger::getInstance().log(tankPressure1->getLastSample());
}

void Sensors::tankPressure2Callback()
{
    Logger::getInstance().log(tankPressure2->getLastSample());
}

void Sensors::servosCurrentCallback()
{
    Logger::getInstance().log(servosCurrent->getLastSample());
}

}  // namespace Motor