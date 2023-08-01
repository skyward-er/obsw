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

#include <Payload/Buses.h>
#include <Payload/Configs/SensorsConfig.h>
#include <common/ReferenceConfig.h>
#include <interfaces-impl/hwmapping.h>

using namespace Boardcore;
using namespace std;
using namespace Payload::SensorsConfig;
namespace Payload
{
LPS22DFData Sensors::getLPS22DFLastSample()
{
    miosix::PauseKernelLock lock;
    return lps22df != nullptr ? lps22df->getLastSample() : LPS22DFData{};
}
LPS28DFWData Sensors::getLPS28DFW_1LastSample()
{
    miosix::PauseKernelLock lock;
    return lps28dfw_1 != nullptr ? lps28dfw_1->getLastSample() : LPS28DFWData{};
}
LPS28DFWData Sensors::getLPS28DFW_2LastSample()
{
    miosix::PauseKernelLock lock;
    return lps28dfw_2 != nullptr ? lps28dfw_2->getLastSample() : LPS28DFWData{};
}
H3LIS331DLData Sensors::getH3LIS331DLLastSample()
{
    miosix::PauseKernelLock lock;
    return h3lis331dl != nullptr ? h3lis331dl->getLastSample()
                                 : H3LIS331DLData{};
}
LIS2MDLData Sensors::getLIS2MDLLastSample()
{
    miosix::PauseKernelLock lock;
    return lis2mdl != nullptr ? lis2mdl->getLastSample() : LIS2MDLData{};
}
UBXGPSData Sensors::getGPSLastSample()
{
    miosix::PauseKernelLock lock;
    return ubxgps != nullptr ? ubxgps->getLastSample() : UBXGPSData{};
}
LSM6DSRXData Sensors::getLSM6DSRXLastSample()
{
    miosix::PauseKernelLock lock;
    return lsm6dsrx != nullptr ? lsm6dsrx->getLastSample() : LSM6DSRXData{};
}
ADS131M08Data Sensors::getADS131M08LastSample()
{
    miosix::PauseKernelLock lock;
    return ads131m08 != nullptr ? ads131m08->getLastSample() : ADS131M08Data{};
}

PitotData Sensors::getPitotLastSample()
{
    miosix::PauseKernelLock lock;
    return pitot != nullptr ? pitot->getLastSample() : PitotData{};
}

Sensors::Sensors(TaskScheduler* sched) : scheduler(sched) {}

bool Sensors::start()
{
    // Init all the sensors
    lps22dfInit();
    lps28dfw_1Init();
    lps28dfw_2Init();
    h3lis331dlInit();
    lis2mdlInit();
    ubxgpsInit();
    lsm6dsrxInit();
    ads131m08Init();
    staticPressureInit();
    pitotPressureInit();
    pitotInit();

    // Create sensor manager with populated map and configured scheduler
    manager = new SensorManager(sensorMap, scheduler);
    return manager->start();
}

void Sensors::stop() { manager->stop(); }

bool Sensors::isStarted()
{
    return manager->areAllSensorsInitialized() && scheduler->isRunning();
}

void Sensors::calibrate() {}

void Sensors::lps22dfInit()
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
    lps22df = new LPS22DF(modules.get<Buses>()->spi3,
                          miosix::sensors::LPS22DF::cs::getPin(), config,
                          sensorConfig);

    // Emplace the sensor inside the map
    SensorInfo info("LPS22DF", LPS22DF_PERIOD,
                    bind(&Sensors::lps22dfCallback, this));
    sensorMap.emplace(make_pair(lps22df, info));
}
void Sensors::lps28dfw_1Init()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Configure the sensor
    LPS28DFW::SensorConfig config{false, LPS28DFW_FSR, LPS28DFW_AVG,
                                  LPS28DFW_ODR, false};

    // Create sensor instance with configured parameters
    lps28dfw_1 = new LPS28DFW(modules.get<Buses>()->i2c1, config);

    // Emplace the sensor inside the map
    SensorInfo info("LPS28DFW_1", LPS28DFW_PERIOD,
                    bind(&Sensors::lps28dfw_1Callback, this));
    sensorMap.emplace(make_pair(lps28dfw_1, info));
}
void Sensors::lps28dfw_2Init()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Configure the sensor
    LPS28DFW::SensorConfig config{true, LPS28DFW_FSR, LPS28DFW_AVG,
                                  LPS28DFW_ODR, false};

    // Create sensor instance with configured parameters
    lps28dfw_2 = new LPS28DFW(modules.get<Buses>()->i2c1, config);

    // Emplace the sensor inside the map
    SensorInfo info("LPS28DFW_2", LPS28DFW_PERIOD,
                    bind(&Sensors::lps28dfw_2Callback, this));
    sensorMap.emplace(make_pair(lps28dfw_2, info));
}
void Sensors::h3lis331dlInit()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Get the correct SPI configuration
    SPIBusConfig config = H3LIS331DL::getDefaultSPIConfig();
    config.clockDivider = SPI::ClockDivider::DIV_16;

    // Create sensor instance with configured parameters
    h3lis331dl = new H3LIS331DL(
        modules.get<Buses>()->spi3, miosix::sensors::H3LIS331DL::cs::getPin(),
        config, H3LIS331DL_ODR, H3LIS331DL_BDU, H3LIS331DL_FSR);

    // Emplace the sensor inside the map
    SensorInfo info("H3LIS331DL", H3LIS331DL_PERIOD,
                    bind(&Sensors::h3lis331dlCallback, this));
    sensorMap.emplace(make_pair(h3lis331dl, info));
}
void Sensors::lis2mdlInit()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Get the correct SPI configuration
    SPIBusConfig config = LIS2MDL::getDefaultSPIConfig();
    config.clockDivider = SPI::ClockDivider::DIV_16;

    // Configure the sensor
    LIS2MDL::Config sensorConfig;
    sensorConfig.deviceMode         = LIS2MDL_OPERATIVE_MODE;
    sensorConfig.odr                = LIS2MDL_ODR;
    sensorConfig.temperatureDivider = LIS2MDL_TEMPERATURE_DIVIDER;

    // Create sensor instance with configured parameters
    lis2mdl = new LIS2MDL(modules.get<Buses>()->spi3,
                          miosix::sensors::LIS2MDL::cs::getPin(), config,
                          sensorConfig);

    // Emplace the sensor inside the map
    SensorInfo info("LIS2MDL", LIS2MDL_PERIOD,
                    bind(&Sensors::lis2mdlCallback, this));
    sensorMap.emplace(make_pair(lis2mdl, info));
}

void Sensors::ubxgpsInit()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Get the correct SPI configuration
    SPIBusConfig config = UBXGPSSpi::getDefaultSPIConfig();
    config.clockDivider = SPI::ClockDivider::DIV_64;

    // Create sensor instance with configured parameters
    ubxgps = new UBXGPSSpi(modules.get<Buses>()->spi4,
                           miosix::sensors::GPS::cs::getPin(), config, 5);

    // Emplace the sensor inside the map
    SensorInfo info("UBXGPS", UBXGPS_PERIOD,
                    bind(&Sensors::ubxgpsCallback, this));
    sensorMap.emplace(make_pair(ubxgps, info));
}

void Sensors::lsm6dsrxInit()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Configure the SPI
    SPIBusConfig config;
    config.clockDivider = SPI::ClockDivider::DIV_32;
    config.mode         = SPI::Mode::MODE_0;

    // Configure the sensor
    LSM6DSRXConfig sensorConfig;
    sensorConfig.bdu = LSM6DSRX_BDU;

    // Accelerometer
    sensorConfig.fsAcc     = LSM6DSRX_ACC_FS;
    sensorConfig.odrAcc    = LSM6DSRX_ACC_ODR;
    sensorConfig.opModeAcc = LSM6DSRX_OPERATING_MODE;

    // Gyroscope
    sensorConfig.fsGyr     = LSM6DSRX_GYR_FS;
    sensorConfig.odrGyr    = LSM6DSRX_GYR_ODR;
    sensorConfig.opModeGyr = LSM6DSRX_OPERATING_MODE;

    // Fifo
    sensorConfig.fifoMode                = LSM6DSRX_FIFO_MODE;
    sensorConfig.fifoTimestampDecimation = LSM6DSRX_FIFO_TIMESTAMP_DECIMATION;
    sensorConfig.fifoTemperatureBdr      = LSM6DSRX_FIFO_TEMPERATURE_BDR;

    // Create sensor instance with configured parameters
    lsm6dsrx = new LSM6DSRX(modules.get<Buses>()->spi1,
                            miosix::sensors::LSM6DSRX::cs::getPin(), config,
                            sensorConfig);

    // Emplace the sensor inside the map
    SensorInfo info("LSM6DSRX", LSM6DSRX_PERIOD,
                    bind(&Sensors::lsm6dsrxCallback, this));
    sensorMap.emplace(make_pair(lsm6dsrx, info));
}

void Sensors::ads131m08Init()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Configure the SPI
    SPIBusConfig config;
    config.clockDivider = SPI::ClockDivider::DIV_32;

    // Configure the device
    ADS131M08::Config sensorConfig;
    sensorConfig.oversamplingRatio     = ADS131M08_OVERSAMPLING_RATIO;
    sensorConfig.globalChopModeEnabled = ADS131M08_GLOBAL_CHOP_MODE;

    // Create the sensor instance with configured parameters
    ads131m08 = new ADS131M08(modules.get<Buses>()->spi4,
                              miosix::sensors::ADS131::cs::getPin(), config,
                              sensorConfig);
    // Emplace the sensor inside the map
    SensorInfo info("ADS131M08", ADS131M08_PERIOD,
                    bind(&Sensors::ads131m08Callback, this));
    sensorMap.emplace(make_pair(ads131m08, info));
}

void Sensors::staticPressureInit()
{
    // create lambda function to read the voltage
    auto readVoltage = (
        [&]() -> ADCData
        {
             auto temp = ads131m08->getLastSample();
            return temp.getVoltage(
               STATIC_PRESSURE_CHANNEL);
        });

    staticPressure = new HSCMRNN015PA(readVoltage);

    // Emplace the sensor inside the map
    SensorInfo info("StaticPressure", ADS131M08_PERIOD,
                    bind(&Sensors::staticPressureCallback, this));
    sensorMap.emplace(make_pair(staticPressure, info));
}

void Sensors::pitotPressureInit()
{
    // create lambda function to read the voltage
    auto readVoltage = (
        [&]() -> ADCData
        {
             auto temp = ads131m08->getLastSample();
            return temp.getVoltage(
                TOTAL_PRESSURE_CHANNEL);
        });

    pitotPressure = new SSCMRNN030PA(readVoltage);

    // Emplace the sensor inside the map
    SensorInfo info("TotalPressure", ADS131M08_PERIOD,
                    bind(&Sensors::pitotPressureCallback, this));
    sensorMap.emplace(make_pair(pitotPressure, info));
}

void Sensors::pitotInit()
{
    // create lambda function to read the pressure
    function<PressureData()> getPitotPressure(
        [&]() { return pitotPressure->getLastSample(); });
    function<float()> getStaticPressure(
        [&]() { return staticPressure->getLastSample().pressure; });

    pitot = new Pitot(getPitotPressure, getStaticPressure);
    pitot->setReferenceValues(Common::ReferenceConfig::defaultReferenceValues);

    // Emplace the sensor inside the map
    SensorInfo info("Pitot", ADS131M08_PERIOD,
                    bind(&Sensors::pitotCallback, this));
    sensorMap.emplace(make_pair(pitotPressure, info));
}

void Sensors::lps22dfCallback()
{
    miosix::PauseKernelLock lock;
    LPS22DFData lastSample = lps22df->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::lps28dfw_1Callback()
{
    miosix::PauseKernelLock lock;
    LPS28DFW_1Data lastSample = lps28dfw_1->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::lps28dfw_2Callback()
{
    miosix::PauseKernelLock lock;
    LPS28DFW_2Data lastSample = lps28dfw_2->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::h3lis331dlCallback()
{
    miosix::PauseKernelLock lock;
    H3LIS331DLData lastSample = h3lis331dl->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::lis2mdlCallback()
{
    miosix::PauseKernelLock lock;
    LIS2MDLData lastSample = lis2mdl->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::ubxgpsCallback()
{
    miosix::PauseKernelLock lock;
    UBXGPSData lastSample = ubxgps->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::lsm6dsrxCallback()
{
    miosix::PauseKernelLock lock;
    LSM6DSRXData lastSample = lsm6dsrx->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::ads131m08Callback()
{
    miosix::PauseKernelLock lock;
    ADS131M08Data lastSample = ads131m08->getLastSample();
    Logger::getInstance().log(lastSample);
}

void Sensors::staticPressureCallback()
{
    miosix::PauseKernelLock lock;
    HSCMRNN015PAData lastSample = staticPressure->getLastSample();
    Logger::getInstance().log(lastSample);
}

void Sensors::pitotPressureCallback()
{
    miosix::PauseKernelLock lock;
    SSCMRNN030PAData lastSample = pitotPressure->getLastSample();
    Logger::getInstance().log(lastSample);
}

void Sensors::pitotCallback()
{
    miosix::PauseKernelLock lock;
    PitotData lastSample = pitot->getLastSample();
    Logger::getInstance().log(lastSample);
}
}  // namespace Payload