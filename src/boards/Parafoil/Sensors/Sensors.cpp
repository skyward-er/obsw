/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <Parafoil/Buses.h>
#include <Parafoil/Configs/SensorsConfig.h>
#include <common/ReferenceConfig.h>
#include <interfaces-impl/hwmapping.h>

using namespace Boardcore;
using namespace std;
using namespace Parafoil::SensorsConfig;

// BMX160 Watermark interrupt
void __attribute__((used)) EXTI5_IRQHandlerImpl()
{
    Parafoil::Sensors* sensors_module =
        ModuleManager::getInstance().get<Parafoil::Sensors>();
    if (sensors_module->bmx160 != nullptr)
        sensors_module->bmx160->IRQupdateTimestamp(
            TimestampTimer::getTimestamp());
}

namespace Parafoil
{
// Getters
BMX160Data Sensors::getBMX160LastSample()
{
    miosix::Lock<FastMutex> l(bmx160Mutex);
    return bmx160 != nullptr ? bmx160->getLastSample() : BMX160Data{};
}
BMX160WithCorrectionData Sensors::getBMX160WithCorrectionLastSample()
{
    miosix::Lock<FastMutex> l(bmx160WithCorrectionMutex);
    return bmx160WithCorrection != nullptr
               ? bmx160WithCorrection->getLastSample()
               : BMX160WithCorrectionData{};
}
LIS3MDLData Sensors::getLIS3MDLLastSample()
{
    miosix::Lock<FastMutex> l(lis3mdlMutex);
    return lis3mdl != nullptr ? lis3mdl->getLastSample() : LIS3MDLData{};
}
MS5803Data Sensors::getMS5803LastSample()
{
    miosix::Lock<FastMutex> l(ms5803Mutex);
    return ms5803 != nullptr ? ms5803->getLastSample() : MS5803Data{};
}
UBXGPSData Sensors::getUbxGpsLastSample()
{
    miosix::Lock<FastMutex> l(ubxGpsMutex);
    return ubxGps != nullptr ? ubxGps->getLastSample() : UBXGPSData{};
}
ADS1118Data Sensors::getADS1118LastSample()
{
    miosix::Lock<FastMutex> l(ads1118Mutex);
    return ads1118 != nullptr ? ads1118->getLastSample() : ADS1118Data{};
}
InternalADCData Sensors::getInternalADCLastSample()
{
    miosix::Lock<FastMutex> l(internalADCMutex);
    return internalADC != nullptr ? internalADC->getLastSample()
                                  : InternalADCData{};
}
BatteryVoltageSensorData Sensors::getBatteryVoltageLastSample()
{
    miosix::Lock<FastMutex> l(batteryVoltageMutex);
    return batteryVoltage != nullptr ? batteryVoltage->getLastSample()
                                     : BatteryVoltageSensorData{};
}

// // Processed Getters
// BatteryVoltageSensorData Sensors::getBatteryVoltageLastSample()
// {
//     // Do not need to pause the kernel, the last sample getter is already
//     // protected
//     ADS131M08Data sample = getADS131M08LastSample();
//     BatteryVoltageSensorData data;

//     // Populate the data
//     data.voltageTimestamp = sample.timestamp;
//     data.channelId        = static_cast<uint8_t>(BATTERY_VOLTAGE_CHANNEL);
//     data.voltage          =
//     sample.getVoltage(BATTERY_VOLTAGE_CHANNEL).voltage; data.batVoltage =
//     sample.getVoltage(BATTERY_VOLTAGE_CHANNEL).voltage *
//                       BATTERY_VOLTAGE_CONVERSION_FACTOR;
//     return data;
// }

Sensors::Sensors(TaskScheduler* sched) : scheduler(sched), sensorsCounter(0) {}

bool Sensors::start()
{
    // Read the magnetometer calibration from predefined file
    magCalibration.fromFile("magCalibration.csv");

    // Init all the sensors
    bmx160Init();
    bmx160WithCorrectionInit();
    lis3mdlInit();
    ms5803Init();
    ubxGpsInit();
    ads1118Init();
    internalADCInit();
    batteryVoltageInit();

    // Add the magnetometer calibration to the scheduler
    size_t result = scheduler->addTask(
        [&]()
        {
            // Gather the last sample data
            MagnetometerData lastSample = getBMX160LastSample();

            // Feed the data to the calibrator inside a protected area.
            // Contention is not high and the use of a mutex is suitable to
            // avoid pausing the kernel for this calibration operation
            {
                miosix::Lock<FastMutex> l(calibrationMutex);
                magCalibrator.feed(lastSample);
            }
        },
        MAG_CALIBRATION_PERIOD);

    // Create sensor manager with populated map and configured scheduler
    manager = new SensorManager(sensorMap, scheduler);
    return manager->start() && result != 0;
}

void Sensors::stop() { manager->stop(); }

bool Sensors::isStarted()
{
    return manager->areAllSensorsInitialized() && scheduler->isRunning();
}

void Sensors::calibrate()
{
    bmx160WithCorrection->startCalibration();

    miosix::Thread::sleep(BMX160_CALIBRATION_DURATION);

    bmx160WithCorrection->stopCalibration();
}

bool Sensors::writeMagCalibration()
{
    // Compute the calibration result in protected area
    {
        miosix::Lock<FastMutex> l(calibrationMutex);
        SixParametersCorrector cal = magCalibrator.computeResult();

        // Check result validity
        if (!isnan(cal.getb()[0]) && !isnan(cal.getb()[1]) &&
            !isnan(cal.getb()[2]) && !isnan(cal.getA()[0]) &&
            !isnan(cal.getA()[1]) && !isnan(cal.getA()[2]))
        {
            magCalibration = cal;

            // Save the calibration to the calibration file
            return magCalibration.toFile("magCalibration.csv");
        }
    }
    return false;
}

// Inits

void Sensors::bmx160Init()
{
    ModuleManager& modules = ModuleManager::getInstance();

    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_8;

    BMX160Config config;
    config.fifoMode      = BMX160Config::FifoMode::HEADER;
    config.fifoWatermark = BMX160_FIFO_WATERMARK;
    config.fifoInterrupt = BMX160Config::FifoInterruptPin::PIN_INT1;

    config.temperatureDivider = BMX160_TEMP_DIVIDER;

    config.accelerometerRange = BMX160_ACC_FSR_ENUM;
    config.gyroscopeRange     = BMX160_GYRO_FSR_ENUM;

    config.accelerometerDataRate = BMX160_ACC_GYRO_ODR_ENUM;
    config.gyroscopeDataRate     = BMX160_ACC_GYRO_ODR_ENUM;
    config.magnetometerRate      = BMX160_MAG_ODR_ENUM;

    config.gyroscopeUnit = BMX160Config::GyroscopeMeasureUnit::RAD;

    bmx160 =
        new BMX160(modules.get<Buses>()->spi1,
                   miosix::sensors::bmx160::cs::getPin(), config, spiConfig);

    SensorInfo info("BMX160", BMX160_SAMPLE_PERIOD,
                    bind(&Sensors::bmx160Callback, this));
    sensorMap.emplace(make_pair(bmx160, info));

    auto bmx160Status =
        ([&]() -> SensorInfo { return manager->getSensorInfo(bmx160); });
    sensorsInit[sensorsCounter++] = bmx160Status;

    LOG_INFO(logger, "BMX160 setup done!");
}
void Sensors::bmx160WithCorrectionInit()
{
    bmx160WithCorrection =
        new BMX160WithCorrection(bmx160, BMX160_AXIS_ROTATION);

    SensorInfo info("BMX160WithCorrection", BMX160_SAMPLE_PERIOD,
                    bind(&Sensors::bmx160WithCorrectionCallback, this));

    sensorMap.emplace(make_pair(bmx160WithCorrection, info));

    auto bmx160WithCorrectionStatus =
        ([&]() -> SensorInfo
         { return manager->getSensorInfo(bmx160WithCorrection); });
    sensorsInit[sensorsCounter++] = bmx160WithCorrectionStatus;

    LOG_INFO(logger, "BMX160WithCorrection setup done!");
}
void Sensors::lis3mdlInit()
{
    ModuleManager& modules = ModuleManager::getInstance();

    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    LIS3MDL::Config config;
    config.odr                = MAG_LIS_ODR;
    config.scale              = MAG_LIS_FULLSCALE;
    config.temperatureDivider = 1;

    lis3mdl =
        new LIS3MDL(modules.get<Buses>()->spi1,
                    miosix::sensors::lis3mdl::cs::getPin(), spiConfig, config);

    // Create the sensor info
    SensorInfo info("LIS3MDL", LIS3MDL_SAMPLE_PERIOD,
                    bind(&Sensors::lis3mdlCallback, this));
    sensorMap.emplace(make_pair(lis3mdl, info));

    auto lis3mdlStatus =
        ([&]() -> SensorInfo { return manager->getSensorInfo(lis3mdl); });
    sensorsInit[sensorsCounter++] = lis3mdlStatus;

    LOG_INFO(logger, "LIS3MDL setup done!");
}
void Sensors::ms5803Init()
{
    ModuleManager& modules = ModuleManager::getInstance();

    SPIBusConfig spiConfig{};
    spiConfig.mode         = SPI::Mode::MODE_3;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    ms5803 = new MS5803(modules.get<Buses>()->spi1,
                        miosix::sensors::ms5803::cs::getPin(), spiConfig,
                        MS5803_TEMP_DIVIDER);

    SensorInfo info("MS5803", MS5803_SAMPLE_PERIOD,
                    bind(&Sensors::ms5803Callback, this));
    sensorMap.emplace(make_pair(ms5803, info));

    auto ms5803Status =
        ([&]() -> SensorInfo { return manager->getSensorInfo(ms5803); });
    sensorsInit[sensorsCounter++] = ms5803Status;

    LOG_INFO(logger, "MS5803 setup done!");
}
void Sensors::ubxGpsInit()
{
    ubxGps =
        new UBXGPSSerial(UBXGPS_BAUD_RATE, UBXGPS_SAMPLE_RATE, USART2, 9600);

    SensorInfo info("UBXGPS", UBXGPS_SAMPLE_PERIOD,
                    bind(&Sensors::ubxGpsCallback, this));
    sensorMap.emplace(make_pair(ubxGps, info));

    auto ubxGpsStatus =
        ([&]() -> SensorInfo { return manager->getSensorInfo(ubxGps); });
    sensorsInit[sensorsCounter++] = ubxGpsStatus;

    LOG_INFO(logger, "UbloxGPS setup done!");
}
void Sensors::ads1118Init()
{
    ModuleManager& modules = ModuleManager::getInstance();

    SPIBusConfig spiConfig = ADS1118::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;

    ADS1118::ADS1118Config config = ADS1118::ADS1118_DEFAULT_CONFIG;
    config.bits.mode              = ADS1118::ADS1118Mode::CONTINUOUS_CONV_MODE;

    ads1118 =
        new ADS1118(modules.get<Buses>()->spi1,
                    miosix::sensors::ads1118::cs::getPin(), config, spiConfig);

    ads1118->enableInput(ADS1118_CH_STATIC_PORT, ADS1118_DR_STATIC_PORT,
                         ADS1118_PGA_STATIC_PORT);
    ads1118->enableInput(ADS1118_CH_PITOT_PORT, ADS1118_DR_PITOT_PORT,
                         ADS1118_PGA_PITOT_PORT);
    ads1118->enableInput(ADS1118_CH_DPL_PORT, ADS1118_DR_DPL_PORT,
                         ADS1118_PGA_DPL_PORT);

    SensorInfo info("ADS1118", ADS1118_SAMPLE_PERIOD,
                    bind(&Sensors::ads1118Callback, this));
    sensorMap.emplace(make_pair(ads1118, info));

    auto ads1118Status =
        ([&]() -> SensorInfo { return manager->getSensorInfo(ads1118); });
    sensorsInit[sensorsCounter++] = ads1118Status;

    LOG_INFO(logger, "ADS1118 adc setup done!");
}
void Sensors::internalADCInit()
{
    internalADC = new InternalADC(ADC3);
    // internalADC = new InternalADC(ADC3, INTERNAL_ADC_VREF);

    internalADC->enableChannel(ADC_BATTERY_VOLTAGE);
    internalADC->enableTemperature();
    internalADC->enableVbat();

    SensorInfo info("INTERNAL_ADC", INTERNAL_ADC_SAMPLE_PERIOD,
                    bind(&Sensors::internalADCCallback, this));

    sensorMap.emplace(std::make_pair(internalADC, info));

    auto internalADCStatus =
        ([&]() -> SensorInfo { return manager->getSensorInfo(internalADC); });
    sensorsInit[sensorsCounter++] = internalADCStatus;

    LOG_INFO(logger, "InternalADC setup done!");
}
void Sensors::batteryVoltageInit()
{
    auto getADCVoltage =
        ([&]() -> ADCData
         { return internalADC->getVoltage(ADC_BATTERY_VOLTAGE); });

    batteryVoltage =
        new BatteryVoltageSensor(getADCVoltage, BATTERY_VOLTAGE_COEFF);

    SensorInfo info("BATTERY_VOLTAGE", INTERNAL_ADC_SAMPLE_PERIOD,
                    bind(&Sensors::batteryVoltageCallback, this));
    sensorMap.emplace(std::make_pair(batteryVoltage, info));

    auto batteryVoltageStatus =
        ([&]() -> SensorInfo
         { return manager->getSensorInfo(batteryVoltage); });
    sensorsInit[sensorsCounter++] = batteryVoltageStatus;

    LOG_INFO(logger, "Battery voltage sensor setup done!");
}

std::array<SensorInfo, 8> Sensors::getSensorInfo()
{
    std::array<SensorInfo, 8> sensorState;
    for (size_t i = 0; i < sensorsInit.size(); i++)
    {
        sensorState[i] = sensorsInit[i]();
    }
    return sensorState;
}

// Callbacks

void Sensors::bmx160Callback()
{
    uint8_t fifoSize = bmx160->getLastFifoSize();
    auto& fifo       = bmx160->getLastFifo();

    Logger::getInstance().log(bmx160->getTemperature());

    for (uint8_t i = 0; i < fifoSize; i++)
        Logger::getInstance().log(fifo.at(i));

    Logger::getInstance().log(bmx160->getFifoStats());
}
void Sensors::bmx160WithCorrectionCallback()
{
    BMX160WithCorrectionData lastSample = bmx160WithCorrection->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::lis3mdlCallback()
{
    LIS3MDLData lastSample = lis3mdl->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::ms5803Callback()
{
    MS5803Data lastSample = ms5803->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::ubxGpsCallback()
{
    UBXGPSData lastSample = ubxGps->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::ads1118Callback()
{
    ADS1118Data lastSample = ads1118->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::internalADCCallback()
{
    InternalADCData lastSample = internalADC->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::batteryVoltageCallback()
{
    BatteryVoltageSensorData lastSample = batteryVoltage->getLastSample();
    Logger::getInstance().log(lastSample);
}

}  // namespace Parafoil
