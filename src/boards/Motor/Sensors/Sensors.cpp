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

using namespace std;
using namespace miosix;
using namespace Boardcore;
using namespace Motor::SensorsConfig;

// LSM6DSRX watermark interrupt
void __attribute__((used)) EXTI7_IRQHandlerImpl()
{
    Motor::Sensors* sensors =
        ModuleManager::getInstance().get<Motor::Sensors>();

    if (sensors->lsm6 != nullptr)
        sensors->lsm6->IRQupdateTimestamp(TimestampTimer::getTimestamp());
}

namespace Motor
{

Sensors::Sensors() {}

Sensors::~Sensors() {}

bool Sensors::start()
{
    adcInit();
    batteryInit();
    lsm6Init();
    h3lisInit();
    lis2Init();
    lps22Init();
    maxInit();
    ads131Init();
    chamberPressureInit();
    tankPressure1Init();
    tankPressure2Init();
    servosCurrentInit();

    sensorManager = new SensorManager(sensorsMap);

    return sensorManager->start();
}

void Sensors::calibrate()
{
    if (ads131 != nullptr)
    {
        ads131->calibrateOffset(ADS131_CHAMBER_PRESSURE_CH);
        ads131->calibrateOffset(ADS131_TANK_PRESSURE_1_CH);
        ads131->calibrateOffset(ADS131_TANK_PRESSURE_2_CH);
        ads131->calibrateOffset(ADS131_SERVO_CURRENT_CH);
    }
}

void Sensors::adcInit()
{
    adc = new InternalADC(ADC1);

    adc->enableTemperature();
    adc->enableVbat();
    adc->enableChannel(ADC_BATTERY_VOLTAGE_CH);

    SensorInfo info("ADC", SAMPLE_PERIOD_ADC,
                    [&]()
                    {
                        // TODO: Log data
                    });

    sensorsMap.emplace(make_pair(adc, info));
}

void Sensors::batteryInit()
{
    function<ADCData()> getADCVoltage(
        bind(&InternalADC::getVoltage, adc, ADC_BATTERY_VOLTAGE_CH));

    battery =
        new BatteryVoltageSensor(getADCVoltage, ADC_BATTERY_VOLTAGE_COEFF);

    SensorInfo info("BATTERY", SAMPLE_PERIOD_ADC,
                    [&]()
                    {
                        // TODO: Log data
                    });

    sensorsMap.emplace(make_pair(battery, info));
}

void Sensors::lsm6Init()
{
    SPIBus& spi1 = ModuleManager::getInstance().get<Buses>()->spi1;

    lsm6 = new LSM6DSRX(spi1, peripherals::lsm6dsrx::cs::getPin(),
                        LSM6_SPI_CONFIG, LSM6_SENSOR_CONFIG);

    miosix::GpioPin interruptPin = peripherals::lsm6dsrx::int2::getPin();
    enableExternalInterrupt(interruptPin.getPort(), interruptPin.getNumber(),
                            InterruptTrigger::FALLING_EDGE, 0);

    SensorInfo info("LSM6", SAMPLE_PERIOD_LSM6,
                    [&]()
                    {
                        // TODO: Log data
                    });

    sensorsMap.emplace(make_pair(lsm6, info));
}

void Sensors::h3lisInit()
{
    SPIBus& spi3 = ModuleManager::getInstance().get<Buses>()->spi3;

    h3lis = new H3LIS331DL(spi3, peripherals::h3lis331dl::cs::getPin(),
                           H3LIS_ODR, H3LIS_BDU, H3LIS_FSR);

    SensorInfo info("H3LIS", SAMPLE_PERIOD_H3LIS,
                    [&]()
                    {
                        // TODO: Log data
                    });

    sensorsMap.emplace(make_pair(h3lis, info));
}

void Sensors::lis2Init()
{
    SPIBus& spi3 = ModuleManager::getInstance().get<Buses>()->spi3;

    lis2 = new LIS2MDL(spi3, peripherals::lis2mdl::cs::getPin(),
                       LIS2_SPI_CONFIG, LIS2_SENSOR_CONFIG);

    SensorInfo info("LIS2", SAMPLE_PERIOD_LIS2,
                    [&]()
                    {
                        // TODO: Log data
                    });

    sensorsMap.emplace(make_pair(lis2, info));
}

void Sensors::lps22Init()
{
    SPIBus& spi3 = ModuleManager::getInstance().get<Buses>()->spi3;

    lps22 = new LPS22DF(spi3, peripherals::lps22df::cs::getPin(),
                        LPS22_SPI_CONFIG, LPS22_SENSOR_CONFIG);

    SensorInfo info("LPS22", SAMPLE_PERIOD_LPS22,
                    [&]()
                    {
                        // TODO: Log data
                    });

    sensorsMap.emplace(make_pair(lps22, info));
}

void Sensors::maxInit()
{
    SPIBus& spi3 = ModuleManager::getInstance().get<Buses>()->spi3;

    max = new MAX31856(spi3, peripherals::max31856::cs::getPin());

    SensorInfo info("MAX", SAMPLE_PERIOD_MAX,
                    [&]()
                    {
                        // TODO: Log data
                    });

    sensorsMap.emplace(make_pair(max, info));
}

void Sensors::ads131Init()
{
    SPIBus& spi4 = ModuleManager::getInstance().get<Buses>()->spi4;

    ads131 = new ADS131M08(spi4, peripherals::ads131m08::cs::getPin(),
                           ADS131_SPI_CONFIG, ADS131_SENSOR_CONFIG);

    SensorInfo info("ADS131", SAMPLE_PERIOD_ADS131,
                    [&]()
                    {
                        // TODO: Log data
                    });

    sensorsMap.emplace(make_pair(ads131, info));
}

void Sensors::chamberPressureInit()
{
    if (ads131 == nullptr)
    {
        return;
    }

    function<ADCData()> getADCVoltage(
        [&](void)
        {
            auto data = ads131->getLastSample();
            return data.getVoltage(ADS131_CHAMBER_PRESSURE_CH);
        });

    chamberPressure =
        new ChamberPressureSensor(getADCVoltage, CHAMBER_PRESSURE_MAX,
                                  CHAMBER_PRESSURE_MIN, CHAMBER_PRESSURE_COEFF);

    SensorInfo info("CHAMBER_PRESSURE", SAMPLE_PERIOD_ADS131,
                    [&]()
                    {
                        // TODO: Log data
                    });

    sensorsMap.emplace(make_pair(chamberPressure, info));
}

void Sensors::tankPressure1Init()
{
    if (ads131 == nullptr)
    {
        return;
    }

    function<ADCData()> getADCVoltage(
        [&](void)
        {
            auto data = ads131->getLastSample();
            return data.getVoltage(ADS131_TANK_PRESSURE_1_CH);
        });

    tankPressure1 =
        new TankPressureSensor1(getADCVoltage, TANK_PRESSURE_1_MAX,
                                TANK_PRESSURE_1_MIN, TANK_PRESSURE_1_COEFF);

    SensorInfo info("TANK_PRESSURE_1", SAMPLE_PERIOD_ADS131,
                    [&]()
                    {
                        // TODO: Log data
                    });

    sensorsMap.emplace(make_pair(tankPressure1, info));
}

void Sensors::tankPressure2Init()
{
    if (ads131 == nullptr)
    {
        return;
    }

    function<ADCData()> getADCVoltage(
        [&](void)
        {
            auto data = ads131->getLastSample();
            return data.getVoltage(ADS131_TANK_PRESSURE_2_CH);
        });

    tankPressure2 =
        new TankPressureSensor2(getADCVoltage, TANK_PRESSURE_2_MAX,
                                TANK_PRESSURE_2_MIN, TANK_PRESSURE_2_COEFF);

    SensorInfo info("TANK_PRESSURE_2", SAMPLE_PERIOD_ADS131,
                    [&]()
                    {
                        // TODO: Log data
                    });

    sensorsMap.emplace(make_pair(tankPressure2, info));
}

void Sensors::servosCurrentInit()
{
    if (ads131 == nullptr)
    {
        return;
    }

    function<ADCData()> getADCVoltage(
        [&](void)
        {
            auto data = ads131->getLastSample();
            return data.getVoltage(ADS131_SERVO_CURRENT_CH);
        });

    function<float(float)> voltageToCurrent(
        [](float voltage) { return voltage * SERVO_CURRENT_COEFF; });

    servosCurrent = new CurrentSensor(getADCVoltage, voltageToCurrent);

    SensorInfo info("SERVOS_CURRENT", SAMPLE_PERIOD_ADS131,
                    [&]()
                    {
                        // TODO: Log data
                    });

    sensorsMap.emplace(make_pair(servosCurrent, info));
}

}  // namespace Motor