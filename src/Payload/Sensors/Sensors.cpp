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

#include <Payload/BoardScheduler.h>
#include <Payload/Buses.h>
#include <Payload/FlightStatsRecorder/FlightStatsRecorder.h>
#include <common/ReferenceConfig.h>
#include <interfaces-impl/hwmapping.h>
#include <sensors/calibration/BiasCalibration/BiasCalibration.h>

using namespace Boardcore;
namespace config = Payload::Config::Sensors;
namespace hwmap  = miosix::sensors;

namespace Payload
{

bool Sensors::start()
{
    // Read the magnetometer calibration data from file
    magCalibration.fromFile(config::MagCalibration::CALIBRATION_PATH);

    if (Config::Sensors::LPS22DF::ENABLED)
        lps22dfInit();

    if (Config::Sensors::LPS28DFW::ENABLED)
        lps28dfwInit();

    if (Config::Sensors::H3LIS331DL::ENABLED)
        h3lis331dlInit();

    if (Config::Sensors::LIS2MDL::ENABLED)
        lis2mdlInit();

    if (Config::Sensors::UBXGPS::ENABLED)
        ubxgpsInit();

    if (Config::Sensors::LSM6DSRX::ENABLED)
        lsm6dsrxInit();

    if (Config::Sensors::ADS131M08::ENABLED)
    {
        ads131m08Init();
        staticPressureInit();
        dynamicPressureInit();
    }

    if (Config::Sensors::InternalADC::ENABLED)
        internalAdcInit();

    if (Config::Sensors::RotatedIMU::ENABLED)
        rotatedImuInit();

    // Return immediately if the hook fails as we cannot know what the hook does
    if (!postSensorCreationHook())
    {
        LOG_ERR(logger, "Sensors post-creation hook failed");
        return false;
    }

    if (!sensorManagerInit())
    {
        LOG_ERR(logger, "Failed to initialize SensorManager");
        return false;
    }

    auto& scheduler = getModule<BoardScheduler>()->sensors();

    magCalibrationTaskId = scheduler.addTask(
        [this]()
        {
            auto mag = getLIS2MDLLastSample();

            miosix::Lock<miosix::FastMutex> lock{magCalibrationMutex};
            magCalibrator.feed(mag);
        },
        config::MagCalibration::SAMPLING_RATE);

    if (magCalibrationTaskId == 0)
    {
        LOG_ERR(logger, "Failed to add mag calibration task");
        return false;
    }

    // Immediately disable the task
    scheduler.disableTask(magCalibrationTaskId);

    started = true;
    return true;
}

bool Sensors::isStarted() { return started; }

void Sensors::calibrate()
{
    BiasCalibration gyroCalibrator{};
    float staticPressureSum  = 0.0f;
    float dynamicPressureSum = 0.0f;
    float lps28dfwSum        = 0.0f;

    using namespace std::chrono;
    auto start = steady_clock::now();

    // Accumulate samples
    for (int i = 0; i < config::Calibration::SAMPLE_COUNT; i++)
    {
        auto lsm6dsrx        = getLSM6DSRXLastSample();
        auto staticPressure  = getStaticPressureLastSample();
        auto dynamicPressure = getDynamicPressureLastSample();
        auto lps28dfw        = getLPS28DFWLastSample();

        gyroCalibrator.feed(static_cast<GyroscopeData>(lsm6dsrx));
        staticPressureSum += staticPressure.pressure;
        dynamicPressureSum += dynamicPressure.pressure;
        lps28dfwSum += lps28dfw.pressure;

        auto wakeup = start + config::Calibration::SAMPLE_PERIOD;
        miosix::Thread::nanoSleepUntil(wakeup.time_since_epoch().count());
    }

    float staticPressureMean =
        staticPressureSum / config::Calibration::SAMPLE_COUNT;
    float dynamicPressureMean =
        dynamicPressureSum / config::Calibration::SAMPLE_COUNT;
    float lps28dfwAvg = lps28dfwSum / config::Calibration::SAMPLE_COUNT;

    // Calibrate all analog pressure sensors against the LPS28DFW or the
    // telemetry reference
    float reference = 0;
    {
        miosix::Lock<miosix::FastMutex> lock{baroCalibrationMutex};
        reference = useBaroCalibrationReference ? baroCalibrationReference
                                                : lps28dfwAvg;
    }

    if (reference > config::Calibration::ATMOS_THRESHOLD)
    {
        // Apply the offset only if reference is valid
        // LPS28DFW might be disabled or unresponsive
        staticPressure->updateOffset(staticPressureMean - reference);
    }
    dynamicPressure->updateOffset(dynamicPressureMean);

    {
        miosix::Lock<miosix::FastMutex> lock{gyroCalibrationMutex};
        gyroCalibration = gyroCalibrator.computeResult();
    }

    // Log the offsets
    auto data = getCalibrationData();
    Logger::getInstance().log(data);
}

SensorCalibrationData Sensors::getCalibrationData()
{
    miosix::Lock<miosix::FastMutex> magLock{magCalibrationMutex};
    miosix::Lock<miosix::FastMutex> gyroLock{gyroCalibrationMutex};

    return SensorCalibrationData{
        .timestamp         = TimestampTimer::getTimestamp(),
        .gyroBiasX         = gyroCalibration.getb().x(),
        .gyroBiasY         = gyroCalibration.getb().y(),
        .gyroBiasZ         = gyroCalibration.getb().z(),
        .magBiasX          = magCalibration.getb().x(),
        .magBiasY          = magCalibration.getb().y(),
        .magBiasZ          = magCalibration.getb().z(),
        .magScaleX         = magCalibration.getA().x(),
        .magScaleY         = magCalibration.getA().y(),
        .magScaleZ         = magCalibration.getA().z(),
        .staticPressBias   = staticPressure->getOffset(),
        .staticPressScale  = 1.0f,
        .dynamicPressBias  = dynamicPressure->getOffset(),
        .dynamicPressScale = 1.0f,
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

void Sensors::setBaroCalibrationReference(float reference)
{
    miosix::Lock<miosix::FastMutex> lock{baroCalibrationMutex};
    baroCalibrationReference    = reference;
    useBaroCalibrationReference = true;
}

void Sensors::resetBaroCalibrationReference()
{
    miosix::Lock<miosix::FastMutex> lock{baroCalibrationMutex};
    baroCalibrationReference    = 0.0f;
    useBaroCalibrationReference = false;
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

StaticPressureData Sensors::getStaticPressureLastSample()
{
    return staticPressure ? StaticPressureData(staticPressure->getLastSample())
                          : StaticPressureData{};
}

DynamicPressureData Sensors::getDynamicPressureLastSample()
{
    return dynamicPressure
               ? DynamicPressureData(dynamicPressure->getLastSample())
               : DynamicPressureData{};
}

IMUData Sensors::getIMULastSample()
{
    return rotatedImu ? rotatedImu->getLastSample() : IMUData{};
}

BatteryVoltageSensorData Sensors::getBatteryVoltage()
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

BatteryVoltageSensorData Sensors::getCamBatteryVoltage()
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

LIS2MDLData Sensors::getCalibratedLIS2MDLLastSample()
{
    auto sample = getLIS2MDLLastSample();

    {
        miosix::Lock<miosix::FastMutex> lock{magCalibrationMutex};
        auto corrected =
            magCalibration.correct(static_cast<MagnetometerData>(sample));
        sample.magneticFieldX = corrected.x();
        sample.magneticFieldY = corrected.y();
        sample.magneticFieldZ = corrected.z();
    }

    return sample;
}

LSM6DSRXData Sensors::getCalibratedLSM6DSRXLastSample()
{
    auto sample = getLSM6DSRXLastSample();

    {
        miosix::Lock<miosix::FastMutex> lock{gyroCalibrationMutex};
        auto corrected =
            gyroCalibration.correct(static_cast<GyroscopeData>(sample));
        sample.angularSpeedX = corrected.x();
        sample.angularSpeedY = corrected.y();
        sample.angularSpeedZ = corrected.z();
    }

    return sample;
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

        PUSH_SENSOR_INFO(lps22df, LPS22DF);
        PUSH_SENSOR_INFO(lps28dfw, LPS28DFW);
        PUSH_SENSOR_INFO(h3lis331dl, H3LIS331DL);
        PUSH_SENSOR_INFO(lis2mdl, LIS2MDL);
        PUSH_SENSOR_INFO(ubxgps, UBXGPS);
        PUSH_SENSOR_INFO(lsm6dsrx, LSM6DSRX);
        PUSH_SENSOR_INFO(ads131m08, ADS131M08);
        PUSH_SENSOR_INFO(internalAdc, InternalADC);
        PUSH_SENSOR_INFO(staticPressure, StaticPressure);
        PUSH_SENSOR_INFO(dynamicPressure, DynamicPressure);
        PUSH_SENSOR_INFO(rotatedImu, RotatedIMU);

#undef PUSH_SENSOR_INFO

        return infos;
    }
    else
    {
        return {};
    }
}  // namespace Payload

void Sensors::lps22dfInit()
{
    auto spiConfig         = LPS22DF::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    auto sensorConfig = LPS22DF::Config{
        .odr = config::LPS22DF::ODR,
        .avg = config::LPS22DF::AVG,
    };

    lps22df = std::make_unique<LPS22DF>(getModule<Buses>()->LPS22DF(),
                                        hwmap::LPS22DF::cs::getPin(), spiConfig,
                                        sensorConfig);
}

void Sensors::lps22dfCallback()
{
    auto sample = getLPS22DFLastSample();
    Logger::getInstance().log(sample);
}

void Sensors::lps28dfwInit()
{
    auto config = LPS28DFW::SensorConfig{
        .sa0  = true,
        .fsr  = config::LPS28DFW::FSR,
        .avg  = config::LPS28DFW::AVG,
        .odr  = config::LPS28DFW::ODR,
        .drdy = false,
    };

    lps28dfw =
        std::make_unique<LPS28DFW>(getModule<Buses>()->LPS28DFW(), config);
}

void Sensors::lps28dfwCallback()
{
    auto sample = getLPS28DFWLastSample();
    // Update pressure stats
    getModule<FlightStatsRecorder>()->updatePressure(sample);
    Logger::getInstance().log(sample);
};

void Sensors::h3lis331dlInit()
{
    auto spiConfig         = H3LIS331DL::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    h3lis331dl = std::make_unique<H3LIS331DL>(
        getModule<Buses>()->H3LIS331DL(), hwmap::H3LIS331DL::cs::getPin(),
        spiConfig, config::H3LIS331DL::ODR, config::H3LIS331DL::BDU,
        config::H3LIS331DL::FSR);
}

void Sensors::h3lis331dlCallback()
{
    auto sample = getH3LIS331DLLastSample();
    // Update acceleration stats
    getModule<FlightStatsRecorder>()->updateAcc(sample);
    Logger::getInstance().log(sample);
}

void Sensors::lis2mdlInit()
{
    auto spiConfig         = LIS2MDL::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    auto sensorConfig = LIS2MDL::Config{
        .odr                = config::LIS2MDL::ODR,
        .deviceMode         = LIS2MDL::MD_CONTINUOUS,
        .temperatureDivider = config::LIS2MDL::TEMPERATURE_DIVIDER,
    };

    lis2mdl = std::make_unique<LIS2MDL>(getModule<Buses>()->LIS2MDL(),
                                        hwmap::LIS2MDL::cs::getPin(), spiConfig,
                                        sensorConfig);
}

void Sensors::lis2mdlCallback()
{
    auto sample = getLIS2MDLLastSample();
    Logger::getInstance().log(sample);
}

void Sensors::ubxgpsInit()
{
    auto spiConfig         = UBXGPSSpi::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;

    ubxgps =
        std::make_unique<UBXGPSSpi>(getModule<Buses>()->UBXGPS(),
                                    hwmap::UBXGps::cs::getPin(), spiConfig, 5);
}

void Sensors::ubxgpsCallback()
{
    auto sample = getUBXGPSLastSample();
    Logger::getInstance().log(sample);
}

void Sensors::lsm6dsrxInit()
{
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

    lsm6dsrx = std::make_unique<LSM6DSRX>(getModule<Buses>()->LSM6DSRX(),
                                          hwmap::LSM6DSRX::cs::getPin(),
                                          spiConfig, sensorConfig);
}

void Sensors::lsm6dsrxCallback()
{
    if (!lsm6dsrx)
        return;

    // We can skip logging the last sample since we are logging the fifo
    auto& logger  = Logger::getInstance();
    auto& fifo    = lsm6dsrx->getLastFifo();
    auto fifoSize = lsm6dsrx->getLastFifoSize();

    // For every instance inside the fifo log the sample
    for (auto i = 0; i < fifoSize; i++)
        logger.log(fifo.at(i));
}

void Sensors::ads131m08Init()
{
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
        channel.enabled = false;
    // Enable required channels
    channels[(int)config::StaticPressure::ADC_CH] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    channels[(int)config::DynamicPressure::ADC_CH] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    ads131m08 = std::make_unique<ADS131M08>(getModule<Buses>()->ADS131M08(),
                                            hwmap::ADS131M08::cs::getPin(),
                                            spiConfig, sensorConfig);
}

void Sensors::ads131m08Callback()
{
    auto sample = getADS131M08LastSample();
    Logger::getInstance().log(sample);
}

void Sensors::internalAdcInit()
{
    internalAdc = std::make_unique<InternalADC>(ADC2);
    internalAdc->enableChannel(config::InternalADC::VBAT_CH);
    internalAdc->enableChannel(config::InternalADC::CAM_VBAT_CH);
    internalAdc->enableTemperature();
    internalAdc->enableVbat();
}

void Sensors::internalAdcCallback()
{
    auto sample = getInternalADCLastSample();
    Logger::getInstance().log(sample);
}

void Sensors::staticPressureInit()
{
    auto readVoltage = [this]
    {
        auto sample  = getADS131M08LastSample();
        auto voltage = sample.getVoltage(config::StaticPressure::ADC_CH);
        voltage.voltage *= config::StaticPressure::SCALE;

        return voltage;
    };

    staticPressure = std::make_unique<MPXH6115A>(readVoltage);
}

void Sensors::staticPressureCallback()
{
    auto sample = getStaticPressureLastSample();
    Logger::getInstance().log(sample);
}

void Sensors::dynamicPressureInit()
{
    auto readVoltage = [this]
    {
        auto sample  = getADS131M08LastSample();
        auto voltage = sample.getVoltage(config::DynamicPressure::ADC_CH);
        voltage.voltage *= config::DynamicPressure::SCALE;

        return voltage;
    };

    dynamicPressure = std::make_unique<MPX5010>(readVoltage);
}

void Sensors::dynamicPressureCallback()
{
    auto sample = getDynamicPressureLastSample();
    Logger::getInstance().log(sample);
}

void Sensors::rotatedImuInit()
{
    rotatedImu = std::make_unique<RotatedIMU>(
        [this]
        {
            auto imu6 = Config::Sensors::RotatedIMU::USE_CALIBRATED_LSM6DSRX
                            ? getCalibratedLSM6DSRXLastSample()
                            : getLSM6DSRXLastSample();
            auto mag  = Config::Sensors::RotatedIMU::USE_CALIBRATED_LIS2MDL
                            ? getCalibratedLIS2MDLLastSample()
                            : getLIS2MDLLastSample();

            return IMUData{imu6, imu6, mag};
        });

    // Accelerometer
    rotatedImu->addAccTransformation(RotatedIMU::rotateAroundZ(+90));
    rotatedImu->addAccTransformation(RotatedIMU::rotateAroundX(+90));
    // Gyroscope
    rotatedImu->addGyroTransformation(RotatedIMU::rotateAroundZ(+90));
    rotatedImu->addGyroTransformation(RotatedIMU::rotateAroundX(+90));
    // Invert the Y axis on the magnetometer
    Eigen::Matrix3f m{{1, 0, 0}, {0, -1, 0}, {0, 0, 1}};
    rotatedImu->addMagTransformation(m);
    // Magnetometer
    rotatedImu->addMagTransformation(RotatedIMU::rotateAroundY(+90));
    rotatedImu->addMagTransformation(RotatedIMU::rotateAroundZ(-90));
}

void Sensors::rotatedImuCallback()
{
    auto sample = getIMULastSample();
    Logger::getInstance().log(sample);
}

bool Sensors::sensorManagerInit()
{
    SensorManager::SensorMap_t map;

    if (lps22df)
    {
        SensorInfo info{"LPS22DF", Config::Sensors::LPS22DF::SAMPLING_RATE,
                        [this]() { lps22dfCallback(); }};
        map.emplace(lps22df.get(), info);
    }

    if (lps28dfw)
    {
        SensorInfo info{"LPS28DFW", Config::Sensors::LPS28DFW::SAMPLING_RATE,
                        [this]() { lps28dfwCallback(); }};
        map.emplace(lps28dfw.get(), info);
    }

    if (h3lis331dl)
    {
        SensorInfo info{"H3LIS331DL",
                        Config::Sensors::H3LIS331DL::SAMPLING_RATE,
                        [this]() { h3lis331dlCallback(); }};
        map.emplace(h3lis331dl.get(), info);
    }

    if (lis2mdl)
    {
        SensorInfo info{"LIS2MDL", Config::Sensors::LIS2MDL::SAMPLING_RATE,
                        [this]() { lis2mdlCallback(); }};
        map.emplace(lis2mdl.get(), info);
    }

    if (ubxgps)
    {
        SensorInfo info{"UBXGPS", Config::Sensors::UBXGPS::SAMPLING_RATE,
                        [this]() { ubxgpsCallback(); }};
        map.emplace(ubxgps.get(), info);
    }

    if (lsm6dsrx)
    {
        SensorInfo info{"LSM6DSRX", Config::Sensors::LSM6DSRX::SAMPLING_RATE,
                        [this]() { lsm6dsrxCallback(); }};
        map.emplace(lsm6dsrx.get(), info);
    }

    if (ads131m08)
    {
        SensorInfo info{"ADS131M08", Config::Sensors::ADS131M08::SAMPLING_RATE,
                        [this]() { ads131m08Callback(); }};
        map.emplace(ads131m08.get(), info);
    }

    if (staticPressure)
    {
        SensorInfo info{"StaticPressure",
                        Config::Sensors::ADS131M08::SAMPLING_RATE,
                        [this]() { staticPressureCallback(); }};
        map.emplace(staticPressure.get(), info);
    }

    if (dynamicPressure)
    {
        SensorInfo info{"DynamicPressure",
                        Config::Sensors::ADS131M08::SAMPLING_RATE,
                        [this]() { dynamicPressureCallback(); }};
        map.emplace(dynamicPressure.get(), info);
    }

    if (internalAdc)
    {
        SensorInfo info{"InternalADC",
                        Config::Sensors::InternalADC::SAMPLING_RATE,
                        [this]() { internalAdcCallback(); }};
        map.emplace(internalAdc.get(), info);
    }

    if (rotatedImu)
    {
        SensorInfo info{"RotatedIMU",
                        Config::Sensors::RotatedIMU::SAMPLING_RATE,
                        [this]() { rotatedImuCallback(); }};
        map.emplace(rotatedImu.get(), info);
    }

    auto& scheduler = getModule<BoardScheduler>()->sensors();
    manager         = std::make_unique<SensorManager>(map, &scheduler);
    return manager->start();
}

}  // namespace Payload
