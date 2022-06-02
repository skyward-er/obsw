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

#include "Sensors.h"

#include <Main/Buses.h>
#include <Main/Configs/SensorsConfig.h>
#include <drivers/interrupt/external_interrupts.h>

using namespace std;
using namespace miosix;
using namespace Boardcore;
using namespace Main::SensorConfigs;

// BMX160 Watermark interrupt
void __attribute__((used)) EXTI5_IRQHandlerImpl()
{
    if (Main::Sensors::getInstance().bmx160 != nullptr)
        Main::Sensors::getInstance().bmx160->IRQupdateTimestamp(
            TimestampTimer::getTimestamp());
}

namespace Main
{

bool Sensors::start()
{
    GpioPin bmx160IntPin = miosix::sensors::bmx160::intr::getPin();
    enableExternalInterrupt(bmx160IntPin.getPort(), bmx160IntPin.getNumber(),
                            InterruptTrigger::FALLING_EDGE);

    return sensorManager->start();
}

Sensors::Sensors()
{
    // Initialize all the sensors
    bmx160Init();
    lis3mdlInit();
    ms5803Init();
    // ubxGpsInit();
    ads1118Init();
    staticPressureInit();
    dplPressureInit();
    pitotPressureInit();
    internalAdcInit();
    batteryVoltageInit();

    // Create the sensor manager
    sensorManager = new SensorManager(sensorsMap);
}

Sensors::~Sensors()
{
    delete bmx160;
    delete lis3mdl;
    delete ms5803;
    delete ubxGps;
    delete ads1118;
    delete staticPressure;
    delete dplPressure;
    delete pitotPressure;
    delete internalAdc;
    delete batteryVoltage;

    sensorManager->stop();
    delete sensorManager;
}

void Sensors::bmx160Init()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    BMX160Config config;
    config.fifoMode      = BMX160Config::FifoMode::HEADER;
    config.fifoWatermark = IMU_BMX_FIFO_WATERMARK;
    config.fifoInterrupt = BMX160Config::FifoInterruptPin::PIN_INT1;

    config.temperatureDivider = IMU_BMX_TEMP_DIVIDER;

    config.accelerometerRange = IMU_BMX_ACC_FULLSCALE_ENUM;
    config.gyroscopeRange     = IMU_BMX_GYRO_FULLSCALE_ENUM;

    config.accelerometerDataRate = IMU_BMX_ACC_GYRO_ODR_ENUM;
    config.gyroscopeDataRate     = IMU_BMX_ACC_GYRO_ODR_ENUM;
    config.magnetometerRate      = IMU_BMX_MAG_ODR_ENUM;

    config.gyroscopeUnit = BMX160Config::GyroscopeMeasureUnit::RAD;

    bmx160 =
        new BMX160(Buses::getInstance().spi1,
                   miosix::sensors::bmx160::cs::getPin(), config, spiConfig);

    SensorInfo info("BMX160", SAMPLE_PERIOD_IMU_BMX,
                    bind(&Sensors::bmx160Callback, this));

    sensorsMap.emplace(std::make_pair(bmx160, info));

    LOG_INFO(logger, "BMX160 Setup done!");
}

void Sensors::bmx160Callback()
{
    uint8_t fifoSize = bmx160->getLastFifoSize();
    auto& fifo       = bmx160->getLastFifo();

    Logger::getInstance().log(bmx160->getTemperature());

    for (uint8_t i = 0; i < fifoSize; i++)
        Logger::getInstance().log(fifo.at(i));

    Logger::getInstance().log(bmx160->getFifoStats());
}

void Sensors::lis3mdlInit()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    LIS3MDL::Config config;
    config.odr                = MAG_LIS_ODR_ENUM;
    config.scale              = MAG_LIS_FULLSCALE;
    config.temperatureDivider = 1;

    lis3mdl =
        new LIS3MDL(Buses::getInstance().spi1,
                    miosix::sensors::lis3mdl::cs::getPin(), spiConfig, config);

    SensorInfo info("LIS3MDL", SAMPLE_PERIOD_MAG_LIS,
                    [&]()
                    { Logger::getInstance().log(lis3mdl->getLastSample()); });

    sensorsMap.emplace(std::make_pair(lis3mdl, info));

    LOG_INFO(logger, "LIS3MDL Setup done!");
}

void Sensors::ms5803Init()
{
    SPIBusConfig spiConfig{};
    spiConfig.mode         = SPI::Mode::MODE_3;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    ms5803 = new MS5803(Buses::getInstance().spi1,
                        miosix::sensors::ms5803::cs::getPin(), spiConfig,
                        TEMP_DIVIDER_PRESS_DIGITAL);

    SensorInfo info("MS5803", SAMPLE_PERIOD_PRESS_DIGITAL,
                    [&]()
                    { Logger::getInstance().log(ms5803->getLastSample()); });

    sensorsMap.emplace(std::make_pair(ms5803, info));

    LOG_INFO(logger, "MS5803 setup done!");
}

void Sensors::ubxGpsInit()
{
    ubxGps = new UBXGPSSerial(GPS_BAUD_RATE, GPS_SAMPLE_RATE);

    SensorInfo info("UBXGPS", GPS_SAMPLE_PERIOD,
                    [&]()
                    { Logger::getInstance().log(ubxGps->getLastSample()); });

    sensorsMap.emplace(std::make_pair(ubxGps, info));

    LOG_INFO(logger, "UBXGPS Setup done!");
}

void Sensors::ads1118Init()
{
    SPIBusConfig spiConfig = ADS1118::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;

    ADS1118::ADS1118Config config = ADS1118::ADS1118_DEFAULT_CONFIG;
    config.bits.mode              = ADS1118::ADS1118Mode::CONTINUOUS_CONV_MODE;

    ads1118 =
        new ADS1118(Buses::getInstance().spi1,
                    miosix::sensors::ads1118::cs::getPin(), config, spiConfig);

    ads1118->enableInput(ADC_CH_STATIC_PORT, ADC_DR_STATIC_PORT,
                         ADC_PGA_STATIC_PORT);
    ads1118->enableInput(ADC_CH_PITOT_PORT, ADC_DR_PITOT_PORT,
                         ADC_PGA_PITOT_PORT);
    ads1118->enableInput(ADC_CH_DPL_PORT, ADC_DR_DPL_PORT, ADC_PGA_DPL_PORT);
    ads1118->enableInput(ADC_CH_VREF, ADC_DR_VREF, ADC_PGA_VREF);

    SensorInfo info("ADS1118", SAMPLE_PERIOD_ADC_ADS1118,
                    [&]()
                    { Logger::getInstance().log(ads1118->getLastSample()); });

    sensorsMap.emplace(std::make_pair(ads1118, info));

    LOG_INFO(logger, "ADS1118 setup done!");
}

void Sensors::staticPressureInit()
{
    function<ADCData()> getVoltage(
        bind(&ADS1118::getVoltage, ads1118, ADC_CH_STATIC_PORT));
    staticPressure = new MPXHZ6130A(getVoltage, REFERENCE_VOLTAGE);

    SensorInfo info(
        "STATIC_PRESSURE", SAMPLE_PERIOD_PRESS_STATIC,
        [&]() { Logger::getInstance().log(staticPressure->getLastSample()); });

    sensorsMap.emplace(std::make_pair(staticPressure, info));

    LOG_INFO(logger, "Static pressure sensor setup done!");
}

void Sensors::dplPressureInit()
{
    function<ADCData()> getVoltage(
        bind(&ADS1118::getVoltage, ads1118, ADC_CH_DPL_PORT));
    dplPressure = new SSCDANN030PAA(getVoltage, REFERENCE_VOLTAGE);

    SensorInfo info(
        "DPL_PRESSURE", SAMPLE_PERIOD_PRESS_DPL,
        [&]() { Logger::getInstance().log(dplPressure->getLastSample()); });

    sensorsMap.emplace(std::make_pair(dplPressure, info));

    LOG_INFO(logger, "Deployment pressure sensor setup done!");
}

void Sensors::pitotPressureInit()
{
    function<ADCData()> getVoltage(
        bind(&ADS1118::getVoltage, ads1118, ADC_CH_PITOT_PORT));
    pitotPressure = new SSCDRRN015PDA(getVoltage, REFERENCE_VOLTAGE);

    SensorInfo info("PITOT", SAMPLE_PERIOD_PRESS_PITOT,
                    bind(&Sensors::pitotPressureCallback, this));

    sensorsMap.emplace(std::make_pair(pitotPressure, info));

    LOG_INFO(logger, "Pitot pressure sensor setup done!");
}

void Sensors::pitotPressureCallback()
{
    SSCDRRN015PDAData pressure = pitotPressure->getLastSample();
    Logger::getInstance().log(pressure);

    // TODO: Implement pitot velocity calculation
}

void Sensors::internalAdcInit()
{
    internalAdc = new InternalADC(ADC3, INTERNAL_ADC_VREF);

    internalAdc->enableChannel(ADC_BATTERY_VOLTAGE_CHANNEL);

    SensorInfo info("INTERNAL_ADC", SAMPLE_PERIOD_INTERNAL_ADC);

    sensorsMap.emplace(std::make_pair(internalAdc, info));

    LOG_INFO(logger, "Internal ADC setup done!");
}

void Sensors::batteryVoltageInit()
{
    function<ADCData()> voltage_fun(bind(&InternalADC::getVoltage, internalAdc,
                                         ADC_BATTERY_VOLTAGE_CHANNEL));
    batteryVoltage =
        new BatteryVoltageSensor(voltage_fun, BATTERY_VOLTAGE_COEFF);

    SensorInfo info(
        "BATTERY_VOLTAGE", SAMPLE_PERIOD_INTERNAL_ADC,
        [&]() { Logger::getInstance().log(batteryVoltage->getLastSample()); });

    sensorsMap.emplace(std::make_pair(batteryVoltage, info));

    LOG_INFO(logger, "Battery voltage sensor setup done!");
}

}  // namespace Main
