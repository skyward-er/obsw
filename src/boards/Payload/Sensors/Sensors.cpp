/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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
#include <common/ReferenceConfig.h>
#include <interfaces-impl/hwmapping.h>

using namespace Boardcore;
namespace config = Payload::Config::Sensors;
namespace hwmap  = miosix::sensors;

namespace
{
/**
 * @brief Logs the last sample of a sensor.
 */
template <class Sensor>
void logSample(Sensor* sensor)
{
    auto sample = sensor->getLastSample();
    Logger::getInstance().log(sample);
}

/**
 * @brief Specialized log function for the LSM6DSRX sensor that also logs its
 * FIFO.
 */
template <>
void logSample(LSM6DSRX* sensor)
{
    auto sample       = sensor->getLastSample();
    auto& fifo        = sensor->getLastFifo();
    uint16_t fifoSize = sensor->getLastFifoSize();

    Logger::getInstance().log(sample);
    // Log every entry in the FIFO
    for (uint16_t i = 0; i < fifoSize; i++)
    {
        Logger::getInstance().log(fifo.at(i));
    }
}
}  // namespace

namespace Payload
{

Sensors::Sensors(TaskScheduler& sched, Buses& buses) : scheduler(sched)
{
    lps22dfCreate(buses);
    lps28dfwCreate(buses);
    h3lis331dlCreate(buses);
    lis2mdlCreate(buses);
    ubxgpsCreate(buses);
    lsm6dsrxCreate(buses);
    ads131m08Create(buses);
    internalAdcCreate(buses);
    staticPressureCreate(buses);
    dynamicPressureCreate(buses);
    pitotCreate(buses);
    imuCreate(buses);
}

bool Sensors::start()
{
    // Read the magnetometer calibration from predefined file
    magCalibration.fromFile("/sd/magCalibration.csv");

    // Insert all sensors in the sensor map
    SensorManager::SensorMap_t map;
    lps22dfInsert(map);
    lps28dfwInsert(map);
    h3lis331dlInsert(map);
    lis2mdlInsert(map);
    ubxgpsInsert(map);
    lsm6dsrxInsert(map);
    ads131m08Insert(map);
    internalAdcInsert(map);
    staticPressureInsert(map);
    dynamicPressureInsert(map);
    pitotInsert(map);
    imuInsert(map);

    bool initResult = magCalibrationInit();

    manager = std::make_unique<SensorManager>(map, &scheduler);
    initResult &= manager->start();

    return initResult;
}

bool Sensors::isStarted()
{
    return manager->areAllSensorsInitialized() && scheduler.isRunning();
}

void Sensors::calibrate()
{
    // Create the stats to calibrate the barometers
    Stats lps28dfwStats;
    Stats staticPressureStats;
    Stats dynamicPressureStats;

    using namespace std::chrono;
    auto start = steady_clock::now();

    // Populate stats with samples
    for (auto i = 0; i < config::Calibration::SAMPLE_COUNT; i++)
    {
        lps28dfwStats.add(getLPS28DFWLastSample().pressure);
        staticPressureStats.add(getStaticPressureLastSample().pressure);
        dynamicPressureStats.add(getDynamicPressureLastSample().pressure);

        auto wakeup = start + config::Calibration::SAMPLE_PERIOD;
        miosix::Thread::nanoSleepUntil(wakeup.time_since_epoch().count());
    }

    // Compute the difference between the mean value from LPS28DFW
    float reference = lps28dfwStats.getStats().mean;

    staticPressure->updateOffset(staticPressureStats.getStats().mean -
                                 reference);
    dynamicPressure->updateOffset(dynamicPressureStats.getStats().mean -
                                  reference);

    // Log the offsets
    auto cal = SensorsCalibrationParameter{
        .timestamp         = TimestampTimer::getTimestamp(),
        .referencePressure = reference,
        .offsetStatic      = staticPressureStats.getStats().mean - reference,
        .offsetDynamic     = dynamicPressureStats.getStats().mean - reference,
    };
    Logger::getInstance().log(cal);
}

bool Sensors::writeMagCalibration()
{
    miosix::Lock<FastMutex> lock(calibrationMutex);
    auto cal = magCalibrator.computeResult();

    using std::isnan;

    // Check result validity
    if (!isnan(cal.getb()[0]) && !isnan(cal.getb()[1]) &&
        !isnan(cal.getb()[2]) && !isnan(cal.getA()[0]) &&
        !isnan(cal.getA()[1]) && !isnan(cal.getA()[2]))
    {
        magCalibration = cal;

        // Save the calibration to the calibration file
        return magCalibration.toFile("/sd/magCalibration.csv");
    }

    return false;
}

LPS22DFData Sensors::getLPS22DFLastSample()
{
    return lps22df ? lps22df->getLastSample() : LPS22DFData{};
}

LPS28DFWData Sensors::getLPS28DFWLastSample()
{
    return lps28dfw ? lps28dfw->getLastSample() : LPS28DFWData{};
}

H3LIS331DLData Sensors::getH3LIS331DLLastSample()
{
    return h3lis331dl ? h3lis331dl->getLastSample() : H3LIS331DLData{};
}

LIS2MDLData Sensors::getLIS2MDLLastSample()
{
    return lis2mdl ? lis2mdl->getLastSample() : LIS2MDLData{};
}

UBXGPSData Sensors::getUBXGPSLastSample()
{
    return ubxgps ? ubxgps->getLastSample() : UBXGPSData{};
}

LSM6DSRXData Sensors::getLSM6DSRXLastSample()
{
    return lsm6dsrx ? lsm6dsrx->getLastSample() : LSM6DSRXData{};
}

ADS131M08Data Sensors::getADS131M08LastSample()
{
    return ads131m08 ? ads131m08->getLastSample() : ADS131M08Data{};
}

InternalADCData Sensors::getInternalADCLastSample()
{
    return internalAdc ? internalAdc->getLastSample() : InternalADCData{};
}

HSCMRNN015PAData Sensors::getStaticPressureLastSample()
{
    return staticPressure ? staticPressure->getLastSample()
                          : HSCMRNN015PAData{};
}

SSCMRNN030PAData Sensors::getDynamicPressureLastSample()
{
    return dynamicPressure ? dynamicPressure->getLastSample()
                           : SSCMRNN030PAData{};
}

PitotData Sensors::getPitotLastSample()
{
    return pitot ? pitot->getLastSample() : PitotData{};
}

RotatedIMUData Sensors::getIMULastSample()
{
    return imu ? imu->getLastSample() : RotatedIMUData{};
}

BatteryVoltageSensorData Sensors::getBatteryVoltageLastSample()
{
    auto sample = getInternalADCLastSample();

    BatteryVoltageSensorData data;
    data.voltageTimestamp = sample.timestamp;
    data.channelId        = static_cast<uint8_t>(config::InternalADC::VBAT_CH);
    data.voltage          = sample.voltage[config::InternalADC::VBAT_CH];
    data.batVoltage       = sample.voltage[config::InternalADC::VBAT_CH] *
                      config::InternalADC::VBAT_SCALE;

    return data;
}

BatteryVoltageSensorData Sensors::getCamBatteryVoltageLastSample()
{
    auto sample = getInternalADCLastSample();

    BatteryVoltageSensorData data;
    data.voltageTimestamp = sample.timestamp;
    data.channelId        = config::InternalADC::CAM_VBAT_CH;
    data.voltage          = sample.voltage[config::InternalADC::CAM_VBAT_CH];
    data.batVoltage       = sample.voltage[config::InternalADC::CAM_VBAT_CH] *
                      config::InternalADC::CAM_VBAT_SCALE;

    return data;
}

MagnetometerData Sensors::getCalibratedMagnetometerLastSample()
{
    MagnetometerData sample = getLIS2MDLLastSample();

    MagnetometerData result;
    // Correct the result with calibration data
    {
        miosix::Lock<FastMutex> lock(calibrationMutex);
        result = static_cast<MagnetometerData>(magCalibration.correct(sample));
    }
    result.magneticFieldTimestamp = sample.magneticFieldTimestamp;

    return result;
}

void Sensors::pitotSetReferenceAltitude(float altitude)
{
    // TODO: reference altitude is unused in the pitot driver
    miosix::PauseKernelLock pkLock;

    ReferenceValues reference = pitot->getReferenceValues();
    reference.refAltitude     = altitude;
    pitot->setReferenceValues(reference);
}

void Sensors::pitotSetReferenceTemperature(float temperature)
{
    // TODO: proper synchronization requires changes in the pitot driver
    miosix::PauseKernelLock pkLock;

    ReferenceValues reference = pitot->getReferenceValues();
    reference.refTemperature  = temperature + 273.15f;
    pitot->setReferenceValues(reference);
}

std::vector<SensorInfo> Sensors::getSensorInfo()
{
    if (manager)
    {
        return {manager->getSensorInfo(lps22df.get()),
                manager->getSensorInfo(lps28dfw.get()),
                manager->getSensorInfo(h3lis331dl.get()),
                manager->getSensorInfo(lis2mdl.get()),
                manager->getSensorInfo(ubxgps.get()),
                manager->getSensorInfo(lsm6dsrx.get()),
                manager->getSensorInfo(ads131m08.get()),
                manager->getSensorInfo(internalAdc.get())};
    }
    else
    {
        return {};
    }
}

void Sensors::lps22dfCreate(Buses& buses)
{
    if (!config::LPS22DF::ENABLED)
    {
        return;
    }

    auto spiConfig         = LPS22DF::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    auto sensorConfig = LPS22DF::Config{
        .odr = config::LPS22DF::ODR,
        .avg = config::LPS22DF::AVG,
    };

    lps22df = std::make_unique<LPS22DF>(
        buses.LPS22DF(), hwmap::LPS22DF::cs::getPin(), spiConfig, sensorConfig);
}

void Sensors::lps22dfInsert(SensorManager::SensorMap_t& map)
{
    if (lps22df)
    {
        auto info = SensorInfo("LPS22DF", config::LPS22DF::SAMPLING_RATE,
                               [this] { logSample(lps22df.get()); });
        map.emplace(lps22df.get(), info);
    }
}

void Sensors::lps28dfwCreate(Buses& buses)
{
    if (!config::LPS28DFW::ENABLED)
    {
        return;
    }

    auto config = LPS28DFW::SensorConfig{
        .sa0  = false,
        .fsr  = config::LPS28DFW::FSR,
        .avg  = config::LPS28DFW::AVG,
        .odr  = config::LPS28DFW::ODR,
        .drdy = false,
    };

    lps28dfw = std::make_unique<LPS28DFW>(buses.LPS28DFW(), config);
}

void Sensors::lps28dfwInsert(SensorManager::SensorMap_t& map)
{
    if (lps28dfw)
    {
        auto info = SensorInfo("LPS28DFW", config::LPS28DFW::SAMPLING_RATE,
                               [this] { logSample(lps28dfw.get()); });
        map.emplace(lps28dfw.get(), info);
    }
}

void Sensors::h3lis331dlCreate(Buses& buses)
{
    if (!config::H3LIS331DL::ENABLED)
    {
        return;
    }

    auto spiConfig         = H3LIS331DL::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    h3lis331dl = std::make_unique<H3LIS331DL>(
        buses.H3LIS331DL(), hwmap::H3LIS331DL::cs::getPin(), spiConfig,
        config::H3LIS331DL::ODR, config::H3LIS331DL::BDU,
        config::H3LIS331DL::FSR);
}

void Sensors::h3lis331dlInsert(SensorManager::SensorMap_t& map)
{
    if (h3lis331dl)
    {
        auto info = SensorInfo("H3LIS331DL", config::H3LIS331DL::SAMPLING_RATE,
                               [this] { logSample(h3lis331dl.get()); });
        map.emplace(h3lis331dl.get(), info);
    }
}

void Sensors::lis2mdlCreate(Buses& buses)
{
    if (!config::LIS2MDL::ENABLED)
    {
        return;
    }

    auto spiConfig         = LIS2MDL::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    auto sensorConfig = LIS2MDL::Config{
        .odr                = config::LIS2MDL::ODR,
        .deviceMode         = config::LIS2MDL::OP_MODE,
        .temperatureDivider = config::LIS2MDL::TEMPERATURE_DIVIDER,
    };

    lis2mdl = std::make_unique<LIS2MDL>(
        buses.LIS2MDL(), hwmap::LIS2MDL::cs::getPin(), spiConfig, sensorConfig);
}

void Sensors::lis2mdlInsert(SensorManager::SensorMap_t& map)
{
    if (lis2mdl)
    {
        auto info = SensorInfo("LIS2MDL", config::LIS2MDL::SAMPLING_RATE,
                               [this] { logSample(lis2mdl.get()); });
        map.emplace(lis2mdl.get(), info);
    }
}

void Sensors::ubxgpsCreate(Buses& buses)
{
    if (!config::UBXGPS::ENABLED)
    {
        return;
    }

    auto spiConfig         = UBXGPSSpi::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;

    ubxgps = std::make_unique<UBXGPSSpi>(
        buses.UBXGPS(), hwmap::UBXGps::cs::getPin(), spiConfig,
        static_cast<uint8_t>(config::UBXGPS::SAMPLE_RATE));
}

void Sensors::ubxgpsInsert(SensorManager::SensorMap_t& map)
{
    if (ubxgps)
    {
        auto info = SensorInfo("UBXGPS", config::UBXGPS::SAMPLING_RATE,
                               [this] { logSample(ubxgps.get()); });
        map.emplace(ubxgps.get(), info);
    }
}

void Sensors::lsm6dsrxCreate(Buses& buses)
{
    if (!config::LSM6DSRX::ENABLED)
    {
        return;
    }

    auto spiConfig         = SPIBusConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;
    spiConfig.mode         = SPI::Mode::MODE_0;

    auto sensorConfig = LSM6DSRXConfig{
        .bdu = LSM6DSRXConfig::BDU::CONTINUOUS_UPDATE,
        // Accelerometer
        .odrAcc    = config::LSM6DSRX::ACC_ODR,
        .opModeAcc = config::LSM6DSRX::OP_MODE,
        .fsAcc     = config::LSM6DSRX::ACC_FS,
        // Gyroscope
        .odrGyr    = config::LSM6DSRX::GYR_ODR,
        .opModeGyr = config::LSM6DSRX::OP_MODE,
        .fsGyr     = config::LSM6DSRX::GYR_FS,
        // Fifo
        .fifoMode = LSM6DSRXConfig::FIFO_MODE::CONTINUOUS,
        .fifoTimestampDecimation =
            LSM6DSRXConfig::FIFO_TIMESTAMP_DECIMATION::DEC_1,
        .fifoTemperatureBdr = LSM6DSRXConfig::FIFO_TEMPERATURE_BDR::DISABLED,
        // Disable interrupts
        .int1InterruptSelection = LSM6DSRXConfig::INTERRUPT::NOTHING,
        .int2InterruptSelection = LSM6DSRXConfig::INTERRUPT::NOTHING,
        // Fifo watermark is unused in continuous mode without interrupts
        .fifoWatermark = 0,
    };

    lsm6dsrx = std::make_unique<LSM6DSRX>(buses.LSM6DSRX(),
                                          hwmap::LSM6DSRX::cs::getPin(),
                                          spiConfig, sensorConfig);
}

void Sensors::lsm6dsrxInsert(SensorManager::SensorMap_t& map)
{
    if (lsm6dsrx)
    {
        auto info = SensorInfo("LSM6DSRX", config::LSM6DSRX::SAMPLING_RATE,
                               [this] { logSample(lsm6dsrx.get()); });
        map.emplace(lsm6dsrx.get(), info);
    }
}

void Sensors::ads131m08Create(Buses& buses)
{
    if (!config::ADS131M08::ENABLED)
    {
        return;
    }

    auto spiConfig         = SPIBusConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    auto sensorConfig = ADS131M08::Config{
        .channelsConfig        = {},
        .oversamplingRatio     = config::ADS131M08::OVERSAMPLING_RATIO,
        .globalChopModeEnabled = config::ADS131M08::GLOBAL_CHOP_MODE,
    };
    auto& channels = sensorConfig.channelsConfig;

    // Disable all channels
    for (auto& channel : channels)
    {
        channel.enabled = false;
    }
    // Enable required channels
    channels[(uint8_t)config::ADS131M08::STATIC_PRESSURE_CH].enabled  = true;
    channels[(uint8_t)config::ADS131M08::DYNAMIC_PRESSURE_CH].enabled = true;

    ads131m08 = std::make_unique<ADS131M08>(buses.ADS131M08(),
                                            hwmap::ADS131M08::cs::getPin(),
                                            spiConfig, sensorConfig);
}

void Sensors::ads131m08Insert(SensorManager::SensorMap_t& map)
{
    if (ads131m08)
    {
        auto info = SensorInfo("ADS131M08", config::ADS131M08::SAMPLING_RATE,
                               [this] { logSample(ads131m08.get()); });
        map.emplace(ads131m08.get(), info);
    }
}

void Sensors::internalAdcCreate(Buses& buses)
{
    if (!config::InternalADC::ENABLED)
    {
        return;
    }

    internalAdc = std::make_unique<InternalADC>(ADC2);
    internalAdc->enableChannel(config::InternalADC::VBAT_CH);
    internalAdc->enableChannel(config::InternalADC::CAM_VBAT_CH);
    internalAdc->enableTemperature();
    internalAdc->enableVbat();
}

void Sensors::internalAdcInsert(SensorManager::SensorMap_t& map)
{
    if (internalAdc)
    {
        auto info =
            SensorInfo("InternalADC", config::InternalADC::SAMPLING_RATE,
                       [this] { logSample(internalAdc.get()); });
        map.emplace(internalAdc.get(), info);
    }
}

void Sensors::staticPressureCreate(Buses& buses)
{
    if (!(config::StaticPressure::ENABLED && config::ADS131M08::ENABLED))
    {
        return;
    }

    auto readVoltage = [this]
    {
        auto sample = ads131m08->getLastSample();
        return sample.getVoltage(config::ADS131M08::STATIC_PRESSURE_CH);
    };

    // TODO: use updated static pressure driver
    staticPressure = std::make_unique<HSCMRNN015PA>(readVoltage, 1.2);
}

void Sensors::staticPressureInsert(SensorManager::SensorMap_t& map)
{
    if (staticPressure)
    {
        auto info =
            SensorInfo("StaticPressure", config::StaticPressure::SAMPLING_RATE,
                       [this] { logSample(staticPressure.get()); });
        map.emplace(staticPressure.get(), info);
    }
}

void Sensors::dynamicPressureCreate(Buses& buses)
{
    if (!(config::DynamicPressure::ENABLED && config::ADS131M08::ENABLED))
    {
        return;
    }

    auto readVoltage = [this]
    {
        auto sample = ads131m08->getLastSample();
        return sample.getVoltage(config::ADS131M08::DYNAMIC_PRESSURE_CH);
    };

    // TODO: use updated dynamic pressure driver
    dynamicPressure = std::make_unique<SSCMRNN030PA>(readVoltage, 1.2);
}

void Sensors::dynamicPressureInsert(SensorManager::SensorMap_t& map)
{
    if (dynamicPressure)
    {
        auto info = SensorInfo("DynamicPressure",
                               config::DynamicPressure::SAMPLING_RATE,
                               [this] { logSample(dynamicPressure.get()); });
        map.emplace(dynamicPressure.get(), info);
    }
}

void Sensors::pitotCreate(Buses& buses)
{
    if (!(config::Pitot::ENABLED && config::StaticPressure::ENABLED &&
          config::DynamicPressure::ENABLED && config::ADS131M08::ENABLED))
    {
        return;
    }

    auto readDynamicPressure = [this]
    { return dynamicPressure->getLastSample().pressure; };

    auto readStaticPressure = [this]
    { return staticPressure->getLastSample().pressure; };

    // TODO: pitot requires total pressure instead of dynamic pressure
    pitot = std::make_unique<Pitot>(readDynamicPressure, readStaticPressure);
    pitot->setReferenceValues(Common::ReferenceConfig::defaultReferenceValues);
}

void Sensors::pitotInsert(SensorManager::SensorMap_t& map)
{
    if (pitot)
    {
        auto info = SensorInfo("Pitot", config::Pitot::SAMPLING_RATE,
                               [this] { logSample(pitot.get()); });
        map.emplace(pitot.get(), info);
    }
}

void Sensors::imuCreate(Buses& buses)
{
    if (!(config::IMU::ENABLED && config::LSM6DSRX::ENABLED &&
          config::LIS2MDL::ENABLED))
    {
        return;
    }

    // TODO: rewrite RotatedIMU completely
    imu = std::make_unique<RotatedIMU>(
        [lsm6dsrx = lsm6dsrx.get()] { return lsm6dsrx->getLastSample(); },
        [this] { return getCalibratedMagnetometerLastSample(); },
        [lsm6dsrx = lsm6dsrx.get()] { return lsm6dsrx->getLastSample(); });

    // Invert the Y axis on the magnetometer
    Eigen::Matrix3f m{{1, 0, 0}, {0, -1, 0}, {0, 0, 1}};
    imu->addMagTransformation(m);
}

void Sensors::imuInsert(SensorManager::SensorMap_t& map)
{
    if (imu)
    {
        auto info = SensorInfo("IMU", config::IMU::SAMPLING_RATE,
                               [this] { logSample(imu.get()); });
        map.emplace(imu.get(), info);
    }
}

bool Sensors::magCalibrationInit()
{
    if (!config::MagCalibration::ENABLED)
    {
        // Nothing to do if mag calibration is disabled
        return true;
    }

    // Add the magnetometer calibration task to the scheduler
    size_t result = scheduler.addTask(
        [this]
        {
            // Gather the last sample data
            auto sample = getLIS2MDLLastSample();

            // Feed the data to the calibrator inside a protected area.
            // Contention is not high and the use of a mutex is suitable to
            // avoid pausing the kernel for this calibration operation
            {
                miosix::Lock<FastMutex> l(calibrationMutex);
                magCalibrator.feed(sample);
            }
        },
        config::MagCalibration::SAMPLING_RATE);

    return result != 0;
}

}  // namespace Payload
