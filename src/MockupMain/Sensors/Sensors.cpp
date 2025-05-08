/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Basso
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

#include <MockupMain/BoardScheduler.h>
#include <MockupMain/Buses.h>
#include <MockupMain/FlightStatsRecorder/FlightStatsRecorder.h>

#include <chrono>

using namespace Boardcore;
using namespace Boardcore::Units::Frequency;
namespace config = MockupMain::Config::Sensors;
namespace hwmap  = miosix::sensors;

namespace MockupMain
{

bool Sensors::start()
{
    if (config::BMX160::ENABLED)
    {
        bmx160Init();
        bmx160WithCorrectionInit();
    }

    if (config::LIS3MDL::ENABLED)
        lis3mdlInit();

    if (config::H3LIS331DL::ENABLED)
        h3lisInit();

    if (config::LPS22DF::ENABLED)
        lps22Init();

    if (config::UBXGPS::ENABLED)
        ubxGpsInit();

    if (config::ADS131M08::ENABLED)
        ads131Init();

    if (config::InternalADC::ENABLED)
        internalADCInit();

    if (!postSensorCreationHook())
    {
        LOG_ERR(logger, "Sensors post-creation hook failed");
        return false;
    }

    if (!sensorManagerInit())
    {
        LOG_ERR(logger, "Failed to initialize sensor manager");
        return false;
    }

    auto& scheduler = getModule<BoardScheduler>()->sensors();

    magCalibrationTaskId = scheduler.addTask(
        [this]()
        {
            auto mag = getLIS3MDLLastSample();

            miosix::Lock<miosix::FastMutex> lock{magCalibrationMutex};
            magCalibrator.feed(mag);
        },
        config::MagCalibration::SAMPLING_RATE);

    if (magCalibrationTaskId == 0)
    {
        LOG_ERR(logger, "Failed to add magnetometer calibration task");
        return false;
    }

    // Immediately disable the task, so that is enabled only when needed
    // from FlightModeManager. It is calibrated during the pre-flight
    // initialization phase.
    scheduler.disableTask(magCalibrationTaskId);

    started = true;
    return true;
}

bool Sensors::isStarted() { return started; }

void Sensors::calibrate()
{
    if (!bmx160WithCorrection)
        return;

    bmx160WithCorrection->startCalibration();

    miosix::Thread::sleep(
        std::chrono::milliseconds{config::BMX160::CALIBRATION_DURATION}
            .count());

    bmx160WithCorrection->stopCalibration();
}

SensorCalibrationData Sensors::getCalibrationData()
{
    miosix::Lock<miosix::FastMutex> lock{magCalibrationMutex};

    return SensorCalibrationData{
        .timestamp = TimestampTimer::getTimestamp(),
        .magBiasX  = magCalibration.getb().x(),
        .magBiasY  = magCalibration.getb().y(),
        .magBiasZ  = magCalibration.getb().z(),
        .magScaleX = magCalibration.getA().x(),
        .magScaleY = magCalibration.getA().y(),
        .magScaleZ = magCalibration.getA().z(),
    };
}

void Sensors::resetMagCalibrator()
{
    miosix::Lock<miosix::FastMutex> lock{magCalibrationMutex};
    magCalibrator = SoftAndHardIronCalibration{};
}

void Sensors::enableMagCalibrator()
{
    getModule<BoardScheduler>()->sensors().enableTask(magCalibrationTaskId);
}

void Sensors::disableMagCalibrator()
{
    getModule<BoardScheduler>()->sensors().disableTask(magCalibrationTaskId);
}

bool Sensors::saveMagCalibration()
{
    miosix::Lock<miosix::FastMutex> lock{magCalibrationMutex};

    SixParametersCorrector calibration = magCalibrator.computeResult();

    // Check if the calibration is valid
    if (!std::isnan(calibration.getb()[0]) &&
        !std::isnan(calibration.getb()[1]) &&
        !std::isnan(calibration.getb()[2]) &&
        !std::isnan(calibration.getA()[0]) &&
        !std::isnan(calibration.getA()[1]) &&
        !std::isnan(calibration.getA()[2]))
    {
        // Its valid, save it and apply it
        magCalibration = calibration;
        return magCalibration.toFile(config::MagCalibration::CALIBRATION_PATH);
    }
    else
    {
        return false;
    }
}

BMX160Data Sensors::getBMX160LastSample()
{
    return bmx160 ? bmx160->getLastSample() : BMX160Data{};
}

BMX160WithCorrectionData Sensors::getBMX160WithCorrectionLastSample()
{
    return bmx160WithCorrection ? bmx160WithCorrection->getLastSample()
                                : BMX160WithCorrectionData{};
}

LIS3MDLData Sensors::getLIS3MDLLastSample()
{
    return lis3mdl ? lis3mdl->getLastSample() : LIS3MDLData{};
}

H3LIS331DLData Sensors::getH3LISLastSample()
{
    return h3lis331dl ? h3lis331dl->getLastSample() : H3LIS331DLData{};
}

LPS22DFData Sensors::getLPS22DFLastSample()
{
    return lps22df ? lps22df->getLastSample() : LPS22DFData{};
}

UBXGPSData Sensors::getUBXGPSLastSample()
{
    return ubxgps ? ubxgps->getLastSample() : UBXGPSData{};
}

ADS131M08Data Sensors::getADS131LastSample()
{
    return ads131m08 ? ads131m08->getLastSample() : ADS131M08Data{};
}

LoadCellData Sensors::getLoadCellLastSample()
{
    return loadCell ? loadCell->getLastSample() : LoadCellData{};
}

InternalADCData Sensors::getInternalADCLastSample()
{
    return internalAdc ? internalAdc->getLastSample() : InternalADCData{};
}

BatteryVoltageSensorData Sensors::getBatteryVoltage()
{
    auto sample = getInternalADCLastSample();

    BatteryVoltageSensorData data;
    data.voltageTimestamp = sample.timestamp;
    data.channelId        = static_cast<uint8_t>(config::InternalADC::VBAT_CH);
    data.voltage          = sample.voltage[config::InternalADC::VBAT_CH];
    data.batVoltage       = sample.voltage[config::InternalADC::VBAT_CH] *
                      config::InternalADC::VBAT_COEFF;

    return data;
}

std::vector<SensorInfo> Sensors::getSensorInfo()
{
    if (manager)
    {
        std::vector<SensorInfo> infos{};

#define PUSH_SENSOR_INFO(instance, name)                         \
    if (instance)                                                \
        infos.push_back(manager->getSensorInfo(instance.get())); \
    else                                                         \
        infos.push_back(                                         \
            SensorInfo { #name, config::name::SAMPLING_RATE, nullptr, false })

        PUSH_SENSOR_INFO(bmx160, BMX160);
        PUSH_SENSOR_INFO(bmx160WithCorrection, BMX160);
        PUSH_SENSOR_INFO(h3lis331dl, H3LIS331DL);
        PUSH_SENSOR_INFO(lis3mdl, LIS3MDL);
        PUSH_SENSOR_INFO(lps22df, LPS22DF);
        PUSH_SENSOR_INFO(ubxgps, UBXGPS);
        PUSH_SENSOR_INFO(ads131m08, ADS131M08);
        PUSH_SENSOR_INFO(internalAdc, InternalADC);

#undef PUSH_SENSOR_INFO

        return infos;
    }
    else
    {
        return {};
    }
}

void Sensors::bmx160Init()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_4;

    BMX160Config config;
    config.fifoMode      = BMX160Config::FifoMode::DISABLED;
    config.fifoWatermark = config::BMX160::FIFO_WATERMARK;
    config.fifoInterrupt = BMX160Config::FifoInterruptPin::PIN_INT1;

    config.temperatureDivider = config::BMX160::TEMP_DIVIDER;

    config.accelerometerRange = config::BMX160::ACC_FSR;
    config.gyroscopeRange     = config::BMX160::GYRO_FSR;

    config.accelerometerDataRate = config::BMX160::ACC_GYRO_ODR;
    config.gyroscopeDataRate     = config::BMX160::ACC_GYRO_ODR;
    config.magnetometerRate      = config::BMX160::MAG_ODR;

    config.gyroscopeUnit = BMX160Config::GyroscopeMeasureUnit::RAD;

    bmx160 = std::make_unique<BMX160>(getModule<Buses>()->spi1,
                                      hwmap::bmx160::cs::getPin(), config,
                                      spiConfig);

    LOG_INFO(logger, "BMX160 initialized!");
}

void Sensors::bmx160WithCorrectionInit()
{
    bmx160WithCorrection = std::make_unique<BMX160WithCorrection>(
        bmx160.get(), config::BMX160::AXIS_ORIENTATION);

    LOG_INFO(logger, "BMX160WithCorrection initialized!");
}

void Sensors::h3lisInit()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    h3lis331dl = std::make_unique<H3LIS331DL>(
        getModule<Buses>()->spi1, hwmap::h3lis331dl::cs::getPin(), spiConfig,
        config::H3LIS331DL::ODR, config::H3LIS331DL::BDU,
        config::H3LIS331DL::FSR);

    LOG_INFO(logger, "H3LIS331DL initialized!");
}

void Sensors::lis3mdlInit()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    LIS3MDL::Config config;
    config.odr                = config::LIS3MDL::ODR;
    config.scale              = config::LIS3MDL::FSR;
    config.temperatureDivider = 1;

    lis3mdl = std::make_unique<LIS3MDL>(getModule<Buses>()->spi1,
                                        hwmap::lis3mdl::cs::getPin(), spiConfig,
                                        config);

    LOG_INFO(logger, "LIS3MDL initialized!");
}

void Sensors::lps22Init()
{
    auto spiConfig         = LPS22DF::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    LPS22DF::Config config;
    config.avg = config::LPS22DF::AVG;
    config.odr = config::LPS22DF::ODR;

    lps22df = std::make_unique<LPS22DF>(getModule<Buses>()->spi1,
                                        hwmap::lps22df::cs::getPin(), spiConfig,
                                        config);

    LOG_INFO(logger, "LPS22DF initialized!");
}

void Sensors::ubxGpsInit()
{
    auto spiConfig         = UBXGPSSpi::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;

    ubxgps = std::make_unique<UBXGPSSpi>(
        getModule<Buses>()->spi1, hwmap::ubxgps::cs::getPin(), spiConfig,
        Kilohertz{config::UBXGPS::SAMPLING_RATE}.value());

    LOG_INFO(logger, "UBXGPS initialized!");
}

void Sensors::ads131Init()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    ADS131M08::Config config;
    config.oversamplingRatio     = config::ADS131M08::OVERSAMPLING_RATIO;
    config.globalChopModeEnabled = config::ADS131M08::GLOBAL_CHOP_MODE;

    ads131m08 = std::make_unique<ADS131M08>(getModule<Buses>()->spi1,
                                            hwmap::ads131m08::cs::getPin(),
                                            spiConfig, config);

    LOG_INFO(logger, "ADS131M08 initialized!");
}

void Sensors::loadCellInit()
{
    loadCell = std::make_unique<AnalogLoadCellSensor>(
        [this]()
        {
            auto sample = getADS131LastSample();
            return sample.getVoltage(config::LoadCell::ADC_CHANNEL);
        },
        config::LoadCell::Calibration::P0_VOLTAGE,
        config::LoadCell::Calibration::P0_MASS,
        config::LoadCell::Calibration::P1_VOLTAGE,
        config::LoadCell::Calibration::P1_MASS);

    LOG_INFO(logger, "LoadCell initialized!");
}

void Sensors::internalADCInit()
{
    internalAdc = std::make_unique<InternalADC>(ADC3);
    internalAdc->enableChannel(config::InternalADC::VBAT_CH);
    internalAdc->enableTemperature();
    internalAdc->enableVbat();

    LOG_INFO(logger, "InternalADC initialized!");
}

void Sensors::bmx160Callback()
{
    if (!bmx160)
        return;

    // We can skip logging the last sample since we are logging the fifo
    auto& logger = Logger::getInstance();
    uint16_t lastFifoSize;
    const auto lastFifo = bmx160->getLastFifo(lastFifoSize);

    // For every instance inside the fifo log the sample
    for (uint16_t i = 0; i < lastFifoSize; i++)
        logger.log(lastFifo.at(i));
}

void Sensors::bmx160WithCorrectionCallback()
{
    BMX160WithCorrectionData lastSample = bmx160WithCorrection->getLastSample();

    // Update acceleration stats
    getModule<FlightStatsRecorder>()->updateAcc(lastSample);

    Logger::getInstance().log(lastSample);
}

void Sensors::h3lisCallback()
{
    H3LIS331DLData lastSample = h3lis331dl->getLastSample();
    Logger::getInstance().log(lastSample);
}

void Sensors::lis3mdlCallback()
{
    LIS3MDLData lastSample = lis3mdl->getLastSample();
    Logger::getInstance().log(lastSample);
}

void Sensors::lps22Callback()
{
    LPS22DFData lastSample = lps22df->getLastSample();
    Logger::getInstance().log(lastSample);
}

void Sensors::ubxGpsCallback()
{
    UBXGPSData lastSample = ubxgps->getLastSample();
    Logger::getInstance().log(lastSample);
}

void Sensors::ads131Callback()
{
    ADS131M08Data lastSample = ads131m08->getLastSample();
    Logger::getInstance().log(lastSample);
}

void Sensors::loadCellCallback()
{
    LoadCellData lastSample = loadCell->getLastSample();
    Logger::getInstance().log(lastSample);
}

void Sensors::internalADCCallback()
{
    InternalADCData lastSample = internalAdc->getLastSample();
    Logger::getInstance().log(lastSample);
}

bool Sensors::sensorManagerInit()
{
    SensorManager::SensorMap_t map;

    if (bmx160)
    {
        SensorInfo info{"BMX160", config::BMX160::SAMPLING_RATE,
                        [this]() { bmx160Callback(); }};
        map.emplace(bmx160.get(), info);
    }

    if (bmx160WithCorrection)
    {
        SensorInfo info{"BMX160WithCorrection", config::BMX160::SAMPLING_RATE,
                        [this]() { bmx160WithCorrectionCallback(); }};
        map.emplace(bmx160WithCorrection.get(), info);
    }

    if (h3lis331dl)
    {
        SensorInfo info{"H3LIS331DL", config::H3LIS331DL::SAMPLING_RATE,
                        [this]() { h3lisCallback(); }};
        map.emplace(h3lis331dl.get(), info);
    }

    if (lis3mdl)
    {
        SensorInfo info{"LIS3MDL", config::LIS3MDL::SAMPLING_RATE,
                        [this]() { lis3mdlCallback(); }};
        map.emplace(lis3mdl.get(), info);
    }

    if (lps22df)
    {
        SensorInfo info{"LPS22DF", config::LPS22DF::SAMPLING_RATE,
                        [this]() { lps22Callback(); }};
        map.emplace(lps22df.get(), info);
    }

    if (ubxgps)
    {
        SensorInfo info{"UBXGPS", config::UBXGPS::SAMPLING_RATE,
                        [this]() { ubxGpsCallback(); }};
        map.emplace(ubxgps.get(), info);
    }

    if (ads131m08)
    {
        SensorInfo info{"ADS131M08", config::ADS131M08::SAMPLING_RATE,
                        [this]() { ads131Callback(); }};
        map.emplace(ads131m08.get(), info);
    }

    if (internalAdc)
    {
        SensorInfo info{"InternalADC", config::InternalADC::SAMPLING_RATE,
                        [this]() { internalADCCallback(); }};
        map.emplace(internalAdc.get(), info);
    }

    auto& scheduler = getModule<BoardScheduler>()->sensors();
    manager         = std::make_unique<SensorManager>(map, &scheduler);
    return manager->start();
}

}  // namespace MockupMain
