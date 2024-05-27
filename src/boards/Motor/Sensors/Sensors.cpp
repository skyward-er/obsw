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

Sensors::Sensors(TaskScheduler* sched, Motor::Buses* buses)
    : scheduler(sched), buses(buses)
{
    // Init all the sensors
    adcCreation();
    batteryCreation();
    h3lis331dlCreation();
    lps22dfCreation();
    max31856Creation();
    ads131m08Creation();
    chamberPressureCreation();
    tankPressure1Creation();
    tankPressure2Creation();
    servosCurrentCreation();
}

Sensors::~Sensors() {}

bool Sensors::start()
{
    if (adc)
    {
        registerSensor(adc.get(), "adc", SAMPLE_PERIOD_ADC,
                       [this]() { this->adcCallback(); });
    }

    if (battery)
    {
        registerSensor(battery.get(), "battery", SAMPLE_PERIOD_ADC,
                       [this]() { this->batteryCallback(); });
    }

    if (h3lis331dl)
    {
        registerSensor(h3lis331dl.get(), "h3lis331dl", SAMPLE_PERIOD_H3LIS,
                       [this]() { this->h3lis331dlCallback(); });
    }

    if (lps22df)
    {
        registerSensor(lps22df.get(), "lps22df", SAMPLE_PERIOD_LPS22,
                       [this]() { this->lps22dfCallback(); });
    }

    if (max31856)
    {
        registerSensor(max31856.get(), "max31856", SAMPLE_PERIOD_MAX,
                       [this]() { this->max31856Callback(); });
    }

    if (ads131m08)
    {
        registerSensor(ads131m08.get(), "ads131m08", SAMPLE_PERIOD_ADS131,
                       [this]() { this->ads131m08Callback(); });
    }
    if (chamberPressure)
    {
        registerSensor(chamberPressure.get(), "chamberPressure",
                       SAMPLE_PERIOD_ADS131,
                       [this]() { this->chamberPressureCallback(); });
    }
    if (tankPressure1)
    {
        registerSensor(tankPressure1.get(), "tankPressure1",
                       SAMPLE_PERIOD_ADS131,
                       [this]() { this->tankPressure1Callback(); });
    }
    if (tankPressure2)
    {
        registerSensor(tankPressure2.get(), " tankPressure2",
                       SAMPLE_PERIOD_ADS131,
                       [this]() { this->tankPressure2Callback(); });
    }
    if (servosCurrent)
    {
        registerSensor(servosCurrent.get(), "servosCurrent",
                       SAMPLE_PERIOD_ADS131,
                       [this]() { this->servosCurrentCallback(); });
    }
    manager = std::make_unique<SensorManager>(sensorsMap, scheduler);

    return manager->start();
}

void Sensors::calibrate()
{
    if (ads131m08 != nullptr)
    {
        ads131m08->calibrateOffset(ADS131_SERVO_CURRENT_CH);
    }
}

void Sensors::adcCreation()
{
    adc = std::make_unique<InternalADC>(ADC1);

    adc->enableTemperature();
    adc->enableVbat();
    adc->enableChannel(ADC_BATTERY_VOLTAGE_CH);
}

void Sensors::batteryCreation()
{
    function<ADCData()> getADCVoltage(
        bind(&InternalADC::getVoltage, adc.get(), ADC_BATTERY_VOLTAGE_CH));

    battery = std::make_unique<BatteryVoltageSensor>(getADCVoltage,
                                                     ADC_BATTERY_VOLTAGE_COEFF);
}

void Sensors::h3lis331dlCreation()
{

    h3lis331dl = std::make_unique<H3LIS331DL>(
        buses->spi3, peripherals::h3lis331dl::cs::getPin(), H3LIS_ODR,
        H3LIS_BDU, H3LIS_FSR);
}

void Sensors::lps22dfCreation()
{

    lps22df = std::make_unique<LPS22DF>(buses->spi3,
                                        peripherals::lps22df::cs::getPin(),
                                        LPS22_SPI_CONFIG, LPS22_SENSOR_CONFIG);
}

void Sensors::max31856Creation()
{
    max31856 = std::make_unique<MAX31856>(buses->spi3,
                                          peripherals::max31856::cs::getPin());
}

void Sensors::ads131m08Creation()
{
    ads131m08 = std::make_unique<ADS131M08>(
        buses->spi4, miosix::peripherals::ads131m08::cs::getPin(),
        ADS131_SPI_CONFIG, ADS131_SENSOR_CONFIG);
}

void Sensors::chamberPressureCreation()
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

    chamberPressure = std::make_unique<ChamberPressureSensor>(
        getADCVoltage, voltageToPressure);
}

void Sensors::tankPressure1Creation()
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

    tankPressure1 =
        std::make_unique<TankPressureSensor1>(getADCVoltage, voltageToPressure);
}

void Sensors::tankPressure2Creation()
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

    tankPressure2 =
        std::make_unique<TankPressureSensor2>(getADCVoltage, voltageToPressure);
}

void Sensors::servosCurrentCreation()
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

    servosCurrent =
        std::make_unique<CurrentSensor>(getADCVoltage, voltageToCurrent);
}

void Sensors::adcCallback() { Logger::getInstance().log(adc->getLastSample()); }

void Sensors::batteryCallback()
{
    Logger::getInstance().log(battery->getLastSample());
}

void Sensors::h3lis331dlCallback()
{
    Logger::getInstance().log(h3lis331dl->getLastSample());
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