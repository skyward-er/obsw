/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Matteo Pignataro, Angelo Prete
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

#include <MockupMain/Buses.h>
#include <MockupMain/Configs/SensorsConfig.h>
#include <common/ReferenceConfig.h>
#include <drivers/interrupt/external_interrupts.h>
#include <interfaces-impl/hwmapping.h>

using namespace Boardcore;
using namespace std;
using namespace MockupMain::SensorsConfig;

// BMX160 Watermark interrupt
void __attribute__((used)) EXTI5_IRQHandlerImpl()
{
    MockupMain::Sensors* sensors_module =
        ModuleManager::getInstance().get<MockupMain::Sensors>();
    if (sensors_module->bmx160 != nullptr)
        sensors_module->bmx160->IRQupdateTimestamp(
            TimestampTimer::getTimestamp());
}

namespace MockupMain
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
H3LIS331DLData Sensors::getH3LISLastSample()
{
    miosix::Lock<FastMutex> l(h3lisMutex);
    return h3lis331dl != nullptr ? h3lis331dl->getLastSample()
                                 : H3LIS331DLData{};
}
LPS22DFData Sensors::getLPS22LastSample()
{
    miosix::Lock<FastMutex> l(lps22Mutex);
    return lps22df != nullptr ? lps22df->getLastSample() : LPS22DFData{};
}
UBXGPSData Sensors::getUbxGpsLastSample()
{
    miosix::Lock<FastMutex> l(ubxGpsMutex);
    return ubxGps != nullptr ? ubxGps->getLastSample() : UBXGPSData{};
}
ADS131M08Data Sensors::getADS131LastSample()
{
    miosix::Lock<FastMutex> l(ads131Mutex);
    return ads131 != nullptr ? ads131->getLastSample() : ADS131M08Data{};
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

LoadCellData Sensors::getLoadCellLastSample()
{
    miosix::Lock<FastMutex> l(loadCellMutex);
    return loadCell != nullptr ? loadCell->getLastSample() : LoadCellData{};
}

// check used task scheduler error due the magnetometer calibration task
Sensors::Sensors(TaskScheduler* sched) : scheduler(sched), sensorsCounter(0) {}

// check calibration of gyro deemed ok by giuseppe
// TODO check axis of bmx

bool Sensors::start()
{
    // Read the magnetometer calibration from predefined file
    miosix::GpioPin cs(GPIOG_BASE, 7);
    cs.mode(miosix::Mode::OUTPUT);
    cs.high();
    // Init all the sensors
    bmx160Init();
    bmx160WithCorrectionInit();
    lis3mdlInit();
    h3lisInit();
    lps22Init();
    ubxGpsInit();
    ads131Init();
    internalADCInit();
    batteryVoltageInit();
    loadCellInit();

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

                // Compute the correction
                SixParametersCorrector cal = magCalibrator.computeResult();

                // Check result validity and save it
                if (!isnan(cal.getb()[0]) && !isnan(cal.getb()[1]) &&
                    !isnan(cal.getb()[2]) && !isnan(cal.getA()[0]) &&
                    !isnan(cal.getA()[1]) && !isnan(cal.getA()[2]))
                {
                    magCalibration = cal;
                }
            }
        },
        MAG_CALIBRATION_PERIOD);

    // Create sensor manager with populated map and configured scheduler
    manager                      = new SensorManager(sensorMap, scheduler);
    miosix::GpioPin interruptPin = miosix::sensors::bmx160::intr::getPin();
    enableExternalInterrupt(interruptPin.getPort(), interruptPin.getNumber(),
                            InterruptTrigger::FALLING_EDGE, 0);
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
    {
        miosix::Lock<FastMutex> l(calibrationMutex);

        // Save the calibration to the calibration file
        return magCalibration.toFile("/sd/bmx160_magnetometer_correction.csv");
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
void Sensors::h3lisInit()
{

    ModuleManager& modules = ModuleManager::getInstance();

    // Get the correct SPI configuration
    SPIBusConfig config = H3LIS331DL::getDefaultSPIConfig();
    config.clockDivider = SPI::ClockDivider::DIV_16;

    // Create sensor instance with configured parameters
    h3lis331dl = new H3LIS331DL(
        modules.get<Buses>()->spi1, miosix::sensors::h3lis331dl::cs::getPin(),
        config, H3LIS331DL_ODR, H3LIS331DL_BDU, H3LIS331DL_FSR);

    // Emplace the sensor inside the map
    SensorInfo info("H3LIS331DL", H3LIS331DL_PERIOD,
                    bind(&Sensors::h3lisCallback, this));
    sensorMap.emplace(make_pair(h3lis331dl, info));

    // used for the sensor state
    auto h3lis331Status =
        ([&]() -> SensorInfo { return manager->getSensorInfo(h3lis331dl); });
    sensorsInit[sensorsCounter++] = h3lis331Status;
}

void Sensors::lps22Init()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Get the correct SPI configuration
    SPIBusConfig config = LPS22DF::getDefaultSPIConfig();
    config.clockDivider = SPI::ClockDivider::DIV_16;

    // Configure the device
    LPS22DF::Config sensorConfig;
    sensorConfig.avg = LPS22DF_AVG;
    sensorConfig.odr = LPS22DF_ODR;

    // Create sensor instance with configured parameters
    lps22df = new LPS22DF(modules.get<Buses>()->spi1,
                          miosix::sensors::lps22df::cs::getPin(), config,
                          sensorConfig);

    // Emplace the sensor inside the map
    SensorInfo info("LPS22DF", LPS22DF_PERIOD,
                    bind(&Sensors::lps22Callback, this));
    sensorMap.emplace(make_pair(lps22df, info));

    // used for the sensor state
    auto lps22Status =
        ([&]() -> SensorInfo { return manager->getSensorInfo(lps22df); });
    sensorsInit[sensorsCounter++] = lps22Status;
}

void Sensors::ubxGpsInit()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Get the correct SPI configuration
    SPIBusConfig config = UBXGPSSpi::getDefaultSPIConfig();
    config.clockDivider = SPI::ClockDivider::DIV_64;

    // Create sensor instance with configured parameters
    ubxGps = new UBXGPSSpi(modules.get<Buses>()->spi1,
                           miosix::sensors::ubxgps::cs::getPin(), config,
                           UBXGPS_SAMPLE_RATE);

    // Emplace the sensor inside the map
    SensorInfo info("UBXGPS", UBXGPS_PERIOD,
                    bind(&Sensors::ubxGpsCallback, this));
    sensorMap.emplace(make_pair(ubxGps, info));

    // used for the sensor state
    auto ubxStatus =
        ([&]() -> SensorInfo { return manager->getSensorInfo(ubxGps); });
    sensorsInit[sensorsCounter++] = ubxStatus;
}
void Sensors::ads131Init()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Configure the SPI
    SPIBusConfig config;
    config.clockDivider = SPI::ClockDivider::DIV_32;

    // Configure the device
    ADS131M08::Config sensorConfig;

    // Disable all channels
    sensorConfig.channelsConfig[0].enabled = false;
    sensorConfig.channelsConfig[1].enabled = false;
    sensorConfig.channelsConfig[2].enabled = false;
    sensorConfig.channelsConfig[3].enabled = false;
    sensorConfig.channelsConfig[4].enabled = false;
    sensorConfig.channelsConfig[5].enabled = false;
    sensorConfig.channelsConfig[6].enabled = false;
    sensorConfig.channelsConfig[7].enabled = false;

    // Configure required channels
    sensorConfig.channelsConfig[(int)SensorsConfig::LOAD_CELL_ADC_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    sensorConfig.oversamplingRatio     = ADS131M08_OVERSAMPLING_RATIO;
    sensorConfig.globalChopModeEnabled = ADS131M08_GLOBAL_CHOP_MODE;

    // Create the sensor instance with configured parameters
    ads131 = new ADS131M08(modules.get<Buses>()->spi1,
                           miosix::sensors::ads131m08::cs::getPin(), config,
                           sensorConfig);

    // Emplace the sensor inside the map
    SensorInfo info("ADS131M08", ADS131M08_PERIOD,
                    bind(&Sensors::ads131Callback, this));
    sensorMap.emplace(make_pair(ads131, info));

    // used for the sensor state
    auto ads131m08Status =
        ([&]() -> SensorInfo { return manager->getSensorInfo(ads131); });
    sensorsInit[sensorsCounter++] = ads131m08Status;
}
void Sensors::internalADCInit()
{
    internalADC = new InternalADC(ADC3);

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
void Sensors::loadCellInit()
{
    loadCell = new AnalogLoadCellSensor(
        [this]()
        {
            auto sample = getADS131LastSample();
            return sample.getVoltage(LOAD_CELL_ADC_CHANNEL);
        },
        LOAD_CELL_P0_VOLTAGE, LOAD_CELL_P0_MASS, LOAD_CELL_P1_VOLTAGE,
        LOAD_CELL_P1_MASS);

    SensorInfo info("LOAD_CELL", LOAD_CELL_SAMPLE_PERIOD,
                    bind(&Sensors::batteryVoltageCallback, this));
    sensorMap.emplace(std::make_pair(batteryVoltage, info));

    auto loadCellStatus =
        ([&]() -> SensorInfo { return manager->getSensorInfo(loadCell); });
    sensorsInit[sensorsCounter++] = loadCellStatus;

    LOG_INFO(logger, "Load cell setup done!");
}

std::array<SensorInfo, NUMBER_OF_SENSORS> Sensors::getSensorInfo()
{
    std::array<SensorInfo, NUMBER_OF_SENSORS> sensorState;
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
void Sensors::h3lisCallback()
{
    H3LIS331DLData lastSample = h3lis331dl->getLastSample();
    Logger::getInstance().log(lastSample);
}

void Sensors::lps22Callback()
{
    LPS22DFData lastSample = lps22df->getLastSample();
    Logger::getInstance().log(lastSample);
}

void Sensors::ubxGpsCallback()
{
    UBXGPSData lastSample = ubxGps->getLastSample();
    Logger::getInstance().log(lastSample);
}

void Sensors::ads131Callback() {
    // We don't log the adc in this test
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
void Sensors::loadCellCallback()
{
    LoadCellData lastSample = loadCell->getLastSample();
    Logger::getInstance().log(lastSample);
}

}  // namespace MockupMain
