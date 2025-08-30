/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Pietro Bortolus
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

#include <Main/Configs/SensorsConfig.h>
#include <interfaces-impl/hwmapping.h>
#include <sensors/calibration/BiasCalibration/BiasCalibration.h>

#include <chrono>

using namespace std::chrono;
using namespace Main;
using namespace Boardcore;
using namespace miosix;
using namespace Eigen;

bool Sensors::isStarted() { return started; }

bool Sensors::start()
{
    // Read the magnetometer calibration from predefined file
    magCalibration.fromFile(Config::Sensors::MAG_CALIBRATION_FILENAME);

    accCalibration0.fromFile(
        Config::Sensors::LSM6DSRX_0::ACC_CALIBRATION_FILENAME);
    gyroCalibration0.fromFile(
        Config::Sensors::LSM6DSRX_0::GYRO_CALIBRATION_FILENAME);

    accCalibration1.fromFile(
        Config::Sensors::LSM6DSRX_1::ACC_CALIBRATION_FILENAME);
    gyroCalibration1.fromFile(
        Config::Sensors::LSM6DSRX_1::GYRO_CALIBRATION_FILENAME);

    if (Config::Sensors::LPS22DF::ENABLED &&
        !Config::Sensors::USING_DUAL_MAGNETOMETER)
        lps22dfInit();

    if (Config::Sensors::H3LIS331DL::ENABLED)
        h3lis331dlInit();

    if (Config::Sensors::LIS2MDL::ENABLED &&
        Config::Sensors::USING_DUAL_MAGNETOMETER)
        lis2mdlExtInit();

    if (Config::Sensors::LIS2MDL::ENABLED)
        lis2mdlInit();

    if (Config::Sensors::UBXGPS::ENABLED)
        ubxgpsInit();

    if (Config::Sensors::LSM6DSRX_0::ENABLED)
        lsm6dsrx0Init();

    if (Config::Sensors::LSM6DSRX_1::ENABLED)
        lsm6dsrx1Init();

    if (Config::Sensors::VN100::ENABLED)
        vn100Init();

    if (Config::Sensors::ND015A::ENABLED)
    {
        nd015a0Init();
        nd015a1Init();
        nd015a2Init();
        nd015a3Init();
    }

    if (Config::Sensors::InternalADC::ENABLED)
        internalAdcInit();

    if (Config::Sensors::IMU::ENABLED)
        rotatedImuInit();

    if (!postSensorCreationHook())
    {
        LOG_ERR(logger, "Failed to call postSensorCreationHook");
        return false;
    }

    if (!sensorManagerInit())
    {
        LOG_ERR(logger, "Failed to init SensorManager");
        return false;
    }

    magCalibrationTaskId = getSensorsScheduler().addTask(
        [this]()
        {
            auto mag = getLIS2MDLExtLastSample();

            Lock<FastMutex> lock{magCalibrationMutex};
            magCalibrator.feed(mag);
        },
        Config::Sensors::MAG_CALIBRATION_RATE);

    if (magCalibrationTaskId == 0)
    {
        LOG_ERR(logger, "Failed to add mag calibration task");
        return false;
    }

    // Immediately disable the task
    getSensorsScheduler().disableTask(magCalibrationTaskId);

    started = true;
    return true;
}

void Sensors::calibrate() {}

CalibrationData Sensors::getCalibration()
{
    // TODO Modify Mavlink messages for ZVK calibration parameters
    Lock<FastMutex> lock1{magCalibrationMutex};

    CalibrationData data;
    data.timestamp = TimestampTimer::getTimestamp();

    data.magBiasX  = magCalibration.getb().x();
    data.magBiasY  = magCalibration.getb().y();
    data.magBiasZ  = magCalibration.getb().z();
    data.magScaleX = magCalibration.getA().x();
    data.magScaleY = magCalibration.getA().y();
    data.magScaleZ = magCalibration.getA().z();

    return data;
}

void Sensors::resetMagCalibrator()
{
    Lock<FastMutex> lock{magCalibrationMutex};
    magCalibrator = SoftAndHardIronCalibration{};
}

void Sensors::enableMagCalibrator()
{
    getSensorsScheduler().enableTask(magCalibrationTaskId);
}

void Sensors::disableMagCalibrator()
{
    getSensorsScheduler().disableTask(magCalibrationTaskId);
}

bool Sensors::saveMagCalibration()
{
    Lock<FastMutex> lock{magCalibrationMutex};

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
        return magCalibration.toFile(Config::Sensors::MAG_CALIBRATION_FILENAME);
    }
    else
    {
        return false;
    }
}

LPS22DFData Sensors::getLPS22DFLastSample()
{
    return lps22df ? lps22df->getLastSample() : LPS22DFData{};
}

H3LIS331DLData Sensors::getH3LIS331DLLastSample()
{
    return h3lis331dl ? h3lis331dl->getLastSample() : H3LIS331DLData{};
}

LIS2MDLData Sensors::getLIS2MDLExtLastSample()
{
    return lis2mdl_ext ? lis2mdl_ext->getLastSample() : LIS2MDLData{};
}

LIS2MDLData Sensors::getLIS2MDLLastSample()
{
    return lis2mdl ? lis2mdl->getLastSample() : LIS2MDLData{};
}

UBXGPSData Sensors::getUBXGPSLastSample()
{
    return ubxgps ? ubxgps->getLastSample() : UBXGPSData{};
}

LSM6DSRXData Sensors::getLSM6DSRX0LastSample()
{
    return lsm6dsrx_0 ? lsm6dsrx_0->getLastSample() : LSM6DSRXData{};
}

LSM6DSRXData Sensors::getLSM6DSRX1LastSample()
{
    return lsm6dsrx_1 ? lsm6dsrx_1->getLastSample() : LSM6DSRXData{};
}

VN100SpiData Sensors::getVN100LastSample()
{
    return vn100 ? vn100->getLastSample() : VN100SpiData{};
}

InternalADCData Sensors::getInternalADCLastSample()
{
    return internalAdc ? internalAdc->getLastSample() : InternalADCData{};
}

VoltageData Sensors::getBatteryVoltageLastSample()
{
    auto sample   = getInternalADCLastSample();
    float voltage = sample.voltage[(int)Config::Sensors::InternalADC::VBAT_CH] *
                    Config::Sensors::InternalADC::VBAT_SCALE;
    return {sample.timestamp, voltage};
}

VoltageData Sensors::getCamBatteryVoltageLastSample()
{
    auto sample = getInternalADCLastSample();
    float voltage =
        sample.voltage[(int)Config::Sensors::InternalADC::CAM_VBAT_CH] *
        Config::Sensors::InternalADC::CAM_VBAT_SCALE;
    return {sample.timestamp, voltage};
}

ND015XData Sensors::getND015A0LastSample()
{
    return nd015a_0 ? nd015a_0->getLastSample() : ND015XData{};
}

ND015XData Sensors::getND015A1LastSample()
{
    return nd015a_1 ? nd015a_1->getLastSample() : ND015XData{};
}

ND015XData Sensors::getND015A2LastSample()
{
    return nd015a_3 ? nd015a_2->getLastSample() : ND015XData{};
}

ND015XData Sensors::getND015A3LastSample()
{
    return nd015a_3 ? nd015a_3->getLastSample() : ND015XData{};
}

LIS2MDLData Sensors::getCalibratedLIS2MDLExtLastSample()
{
    auto sample = getLIS2MDLExtLastSample();

    {
        Lock<FastMutex> lock{magCalibrationMutex};
        auto corrected =
            magCalibration.correct(static_cast<MagnetometerData>(sample));
        sample.magneticFieldX = corrected.x();
        sample.magneticFieldY = corrected.y();
        sample.magneticFieldZ = corrected.z();
    }

    return sample;
}

LIS2MDLData Sensors::getCalibratedLIS2MDLLastSample()
{
    auto sample = getLIS2MDLLastSample();

    {
        Lock<FastMutex> lock{magCalibrationMutex};
        auto corrected =
            magCalibration.correct(static_cast<MagnetometerData>(sample));
        sample.magneticFieldX = corrected.x();
        sample.magneticFieldY = corrected.y();
        sample.magneticFieldZ = corrected.z();
    }

    return sample;
}

LSM6DSRXData Sensors::getCalibratedLSM6DSRX0LastSample()
{
    auto sample = getLSM6DSRX0LastSample();
    Lock<FastMutex> lock{lsm6Calibration0Mutex};

    auto correctedAcc =
        accCalibration0.correct(static_cast<AccelerometerData>(sample));
    sample.accelerationX = correctedAcc.x();
    sample.accelerationY = correctedAcc.y();
    sample.accelerationZ = correctedAcc.z();

    auto correctedGyro =
        gyroCalibration0.correct(static_cast<GyroscopeData>(sample));
    sample.angularSpeedX = correctedGyro.x();
    sample.angularSpeedY = correctedGyro.y();
    sample.angularSpeedZ = correctedGyro.z();

    return sample;
}

LSM6DSRXData Sensors::getCalibratedLSM6DSRX1LastSample()
{
    auto sample = getLSM6DSRX1LastSample();
    Lock<FastMutex> lock{lsm6Calibration1Mutex};

    auto correctedAcc =
        accCalibration1.correct(static_cast<AccelerometerData>(sample));
    sample.accelerationX = correctedAcc.x();
    sample.accelerationY = correctedAcc.y();
    sample.accelerationZ = correctedAcc.z();

    auto correctedGyro =
        gyroCalibration1.correct(static_cast<GyroscopeData>(sample));
    sample.angularSpeedX = correctedGyro.x();
    sample.angularSpeedY = correctedGyro.y();
    sample.angularSpeedZ = correctedGyro.z();

    return sample;
}

IMUData Sensors::getIMULastSample()
{
    return rotatedImu ? rotatedImu->getLastSample() : IMUData{};
}

PressureData Sensors::getAtmosPressureLastSample(
    Config::Sensors::Atmos::AtmosSensor sensor)
{
    switch (sensor)
    {
        case (Config::Sensors::Atmos::AtmosSensor::SENSOR_0):
            return getND015A0LastSample();
        case (Config::Sensors::Atmos::AtmosSensor::SENSOR_1):
            return getND015A1LastSample();
        case (Config::Sensors::Atmos::AtmosSensor::SENSOR_2):
            return getND015A2LastSample();
        default:
        {
            LOG_ERR(logger,
                    "Invalid ATMOS_SENSOR value, using ND015A_0 sensor");
            return getND015A0LastSample();
        }
    }
}

PressureData Sensors::getDplBayPressureLastSample()
{
    return getND015A3LastSample();
}

TemperatureData Sensors::getTemperatureLastSample()
{
    return getLSM6DSRX0LastSample();
}

PressureData Sensors::getCanPitotDynamicPressure()
{
    Lock<FastMutex> lock{canMutex};
    return canPitotDynamicPressure;
}

PressureData Sensors::getCanPitotStaticPressure()
{
    Lock<FastMutex> lock{canMutex};
    return canPitotStaticPressure;
}

void Sensors::setCanPitotDynamicPressure(PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canPitotDynamicPressure = data;
}

void Sensors::setCanPitotStaticPressure(PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canPitotStaticPressure = data;
}

std::vector<SensorInfo> Sensors::getSensorInfos()
{
    if (manager)
    {
        std::vector<SensorInfo> infos{};

#define PUSH_SENSOR_INFO(instance, name)                         \
    if (instance)                                                \
        infos.push_back(manager->getSensorInfo(instance.get())); \
    else                                                         \
        infos.push_back(SensorInfo{name, 0, 0, SensorStatus::NOT_INIT})

        PUSH_SENSOR_INFO(lps22df, "LPS22DF");
        PUSH_SENSOR_INFO(lis2mdl_ext, "LIS2MDL_EXT");
        PUSH_SENSOR_INFO(lis2mdl, "LIS2MDL");
        PUSH_SENSOR_INFO(h3lis331dl, "H3LIS331DL");
        PUSH_SENSOR_INFO(ubxgps, "UBXGPS");
        PUSH_SENSOR_INFO(lsm6dsrx_0, "LSM6DSRX_0");
        PUSH_SENSOR_INFO(lsm6dsrx_1, "LSM6DSRX_1");
        PUSH_SENSOR_INFO(vn100, "VN100");
        PUSH_SENSOR_INFO(internalAdc, "InternalADC");
        PUSH_SENSOR_INFO(nd015a_0, "ND015A_0");
        PUSH_SENSOR_INFO(nd015a_1, "ND015A_1");
        PUSH_SENSOR_INFO(nd015a_2, "ND015A_2");
        PUSH_SENSOR_INFO(nd015a_3, "ND015A_3");
        PUSH_SENSOR_INFO(rotatedImu, "RotatedIMU");

        return infos;
    }
    else
    {
        return {};
    }
}

TaskScheduler& Sensors::getSensorsScheduler()
{
    return getModule<BoardScheduler>()->getSensorsScheduler();
}

void Sensors::lps22dfInit()
{
    SPIBusConfig spiConfig = LPS22DF::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    LPS22DF::Config config;
    config.avg = Config::Sensors::LPS22DF::AVG;
    config.odr = Config::Sensors::LPS22DF::ODR;

    lps22df = std::make_unique<LPS22DF>(getModule<Buses>()->getLPS22DF(),
                                        sensors::LPS22DF::cs::getPin(),
                                        spiConfig, config);
}

void Sensors::lps22dfCallback() { sdLogger.log(getLPS22DFLastSample()); }

void Sensors::h3lis331dlInit()
{
    SPIBusConfig spiConfig = H3LIS331DL::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    h3lis331dl = std::make_unique<H3LIS331DL>(
        getModule<Buses>()->getH3LIS331DL(), sensors::H3LIS331DL::cs::getPin(),
        spiConfig, Config::Sensors::H3LIS331DL::ODR,
        H3LIS331DLDefs::BlockDataUpdate::BDU_CONTINUOS_UPDATE,
        Config::Sensors::H3LIS331DL::FS);
}

void Sensors::h3lis331dlCallback()
{
    auto sample = getH3LIS331DLLastSample();

    // Also update StatsRecorder
    getModule<StatsRecorder>()->updateAcc(sample);
    sdLogger.log(sample);
}

void Sensors::lis2mdlExtInit()
{
    SPIBusConfig spiConfig = LIS2MDL::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    LIS2MDL::Config config;
    config.deviceMode         = LIS2MDL::MD_CONTINUOUS;
    config.odr                = Config::Sensors::LIS2MDL::ODR;
    config.temperatureDivider = Config::Sensors::LIS2MDL::TEMP_DIVIDER;

    lis2mdl_ext = std::make_unique<LIS2MDL>(getModule<Buses>()->getLIS2MDL(),
                                            sensors::LIS2MDL_EXT::cs::getPin(),
                                            spiConfig, config);
}

void Sensors::lis2mdlExtCallback()
{
    sdLogger.log(LIS2MDLExternalData{getLIS2MDLExtLastSample()});
}

void Sensors::lis2mdlInit()
{
    SPIBusConfig spiConfig = LIS2MDL::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    LIS2MDL::Config config;
    config.deviceMode         = LIS2MDL::MD_CONTINUOUS;
    config.odr                = Config::Sensors::LIS2MDL::ODR;
    config.temperatureDivider = Config::Sensors::LIS2MDL::TEMP_DIVIDER;

    lis2mdl = std::make_unique<LIS2MDL>(getModule<Buses>()->getLIS2MDL(),
                                        sensors::LIS2MDL::cs::getPin(),
                                        spiConfig, config);
}

void Sensors::lis2mdlCallback() { sdLogger.log(getLIS2MDLLastSample()); }

void Sensors::ubxgpsInit()
{
    SPIBusConfig spiConfig = UBXGPSSpi::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;

    ubxgps = std::make_unique<UBXGPSSpi>(getModule<Buses>()->getUBXGps(),
                                         sensors::UBXGps::cs::getPin(),
                                         spiConfig, 5);
}

void Sensors::ubxgpsCallback() { sdLogger.log(getUBXGPSLastSample()); }

void Sensors::lsm6dsrx0Init()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;
    spiConfig.mode         = SPI::Mode::MODE_0;

    LSM6DSRXConfig config;
    config.bdu = LSM6DSRXConfig::BDU::CONTINUOUS_UPDATE;

    config.fsAcc     = Config::Sensors::LSM6DSRX_0::ACC_FS;
    config.odrAcc    = Config::Sensors::LSM6DSRX_0::ACC_ODR;
    config.opModeAcc = Config::Sensors::LSM6DSRX_0::ACC_OP_MODE;

    config.fsGyr     = Config::Sensors::LSM6DSRX_0::GYR_FS;
    config.odrGyr    = Config::Sensors::LSM6DSRX_0::GYR_ODR;
    config.opModeGyr = Config::Sensors::LSM6DSRX_0::GYR_OP_MODE;

    config.fifoMode = LSM6DSRXConfig::FIFO_MODE::CONTINUOUS;
    config.fifoTimestampDecimation =
        LSM6DSRXConfig::FIFO_TIMESTAMP_DECIMATION::DEC_1;
    config.fifoTemperatureBdr = LSM6DSRXConfig::FIFO_TEMPERATURE_BDR::HZ_52;

    lsm6dsrx_0 = std::make_unique<LSM6DSRX>(getModule<Buses>()->getLSM6DSRX(),
                                            sensors::LSM6DSRX_0::cs::getPin(),
                                            spiConfig, config);
}

void Sensors::lsm6dsrx0Callback()
{
    if (!lsm6dsrx_0)
        return;

    // For every instance inside the fifo log the sample
    uint16_t lastFifoSize;
    const auto lastFifo = lsm6dsrx_0->getLastFifo(lastFifoSize);
    for (uint16_t i = 0; i < lastFifoSize; i++)
        sdLogger.log(LSM6DSRX0Data{lastFifo.at(i)});
}

void Sensors::lsm6dsrx1Init()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;
    spiConfig.mode         = SPI::Mode::MODE_0;

    LSM6DSRXConfig config;
    config.bdu = LSM6DSRXConfig::BDU::CONTINUOUS_UPDATE;

    config.fsAcc     = Config::Sensors::LSM6DSRX_1::ACC_FS;
    config.odrAcc    = Config::Sensors::LSM6DSRX_1::ACC_ODR;
    config.opModeAcc = Config::Sensors::LSM6DSRX_1::ACC_OP_MODE;

    config.fsGyr     = Config::Sensors::LSM6DSRX_1::GYR_FS;
    config.odrGyr    = Config::Sensors::LSM6DSRX_1::GYR_ODR;
    config.opModeGyr = Config::Sensors::LSM6DSRX_1::GYR_OP_MODE;

    config.fifoMode = LSM6DSRXConfig::FIFO_MODE::CONTINUOUS;
    config.fifoTimestampDecimation =
        LSM6DSRXConfig::FIFO_TIMESTAMP_DECIMATION::DEC_1;
    config.fifoTemperatureBdr = LSM6DSRXConfig::FIFO_TEMPERATURE_BDR::HZ_52;

    lsm6dsrx_1 = std::make_unique<LSM6DSRX>(getModule<Buses>()->getLSM6DSRX(),
                                            sensors::LSM6DSRX_1::cs::getPin(),
                                            spiConfig, config);
}

void Sensors::lsm6dsrx1Callback()
{
    if (!lsm6dsrx_1)
        return;

    // For every instance inside the fifo log the sample
    uint16_t lastFifoSize;
    const auto lastFifo = lsm6dsrx_1->getLastFifo(lastFifoSize);
    for (uint16_t i = 0; i < lastFifoSize; i++)
        sdLogger.log(LSM6DSRX1Data{lastFifo.at(i)});
}

void Sensors::vn100Init()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;
    spiConfig.mode         = SPI::Mode::MODE_3;

    vn100 = std::make_unique<VN100Spi>(getModule<Buses>()->getVN100(),
                                       sensors::VN100::cs::getPin(), spiConfig,
                                       200);
}

void Sensors::vn100Callback() { sdLogger.log(getVN100LastSample()); }

void Sensors::internalAdcInit()
{
    internalAdc = std::make_unique<InternalADC>(ADC2);
    internalAdc->enableChannel(Config::Sensors::InternalADC::VBAT_CH);
    internalAdc->enableChannel(Config::Sensors::InternalADC::CAM_VBAT_CH);
    internalAdc->enableTemperature();
    internalAdc->enableVbat();
}

void Sensors::internalAdcCallback()
{
    sdLogger.log(getInternalADCLastSample());
}

void Sensors::nd015a0Init()
{
    SPIBusConfig spiConfig = ND015A::getDefaultSPIConfig();

    nd015a_0 = std::make_unique<ND015A>(
        getModule<Buses>()->getND015A(), sensors::ND015A_0::cs::getPin(),
        spiConfig, Config::Sensors::ND015A::IOW, Config::Sensors::ND015A::BWL,
        Config::Sensors::ND015A::NTC, Config::Sensors::ND015A::ODR);
}

void Sensors::nd015a0Callback()
{
    sdLogger.log(StaticPressure0Data{getND015A0LastSample()});
}

void Sensors::nd015a1Init()
{
    SPIBusConfig spiConfig = ND015A::getDefaultSPIConfig();

    nd015a_1 = std::make_unique<ND015A>(
        getModule<Buses>()->getND015A(), sensors::ND015A_1::cs::getPin(),
        spiConfig, Config::Sensors::ND015A::IOW, Config::Sensors::ND015A::BWL,
        Config::Sensors::ND015A::NTC, Config::Sensors::ND015A::ODR);
}

void Sensors::nd015a1Callback()
{
    sdLogger.log(StaticPressure1Data{getND015A1LastSample()});
}

void Sensors::nd015a2Init()
{
    SPIBusConfig spiConfig = ND015A::getDefaultSPIConfig();

    nd015a_2 = std::make_unique<ND015A>(
        getModule<Buses>()->getND015A(), sensors::ND015A_2::cs::getPin(),
        spiConfig, Config::Sensors::ND015A::IOW, Config::Sensors::ND015A::BWL,
        Config::Sensors::ND015A::NTC, Config::Sensors::ND015A::ODR);
}

void Sensors::nd015a2Callback()
{
    sdLogger.log(StaticPressure2Data{getND015A2LastSample()});
}

void Sensors::nd015a3Init()
{
    SPIBusConfig spiConfig = ND015A::getDefaultSPIConfig();

    nd015a_3 = std::make_unique<ND015A>(
        getModule<Buses>()->getND015A(), sensors::ND015A_3::cs::getPin(),
        spiConfig, Config::Sensors::ND015A::IOW, Config::Sensors::ND015A::BWL,
        Config::Sensors::ND015A::NTC, Config::Sensors::ND015A::ODR);
}

void Sensors::nd015a3Callback()
{
    sdLogger.log(DplBayPressureData{getND015A3LastSample()});
}

void Sensors::rotatedImuInit()
{
    rotatedImu = std::make_unique<RotatedIMU>(
        [this]()
        {
            auto imu6 = Config::Sensors::IMU::USE_CALIBRATED_LSM6DSRX
                            ? getCalibratedLSM6DSRX0LastSample()
                            : getLSM6DSRX0LastSample();
            auto mag  = Config::Sensors::IMU::USE_CALIBRATED_LIS2MDL
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
    Matrix3f m{{1, 0, 0}, {0, -1, 0}, {0, 0, 1}};
    rotatedImu->addMagTransformation(m);
    // Magnetometer
    rotatedImu->addMagTransformation(RotatedIMU::rotateAroundY(+90));
    rotatedImu->addMagTransformation(RotatedIMU::rotateAroundZ(-90));
}

void Sensors::rotatedImuCallback() { sdLogger.log(getIMULastSample()); }

bool Sensors::sensorManagerInit()
{
    SensorManager::SensorMap_t map;

    if (lps22df)
    {
        SensorConfig config{"LPS22DF", Config::Sensors::LPS22DF::RATE,
                        [this]() { lps22dfCallback(); }, 0};
        map.emplace(lps22df.get(), config);
    }

    if (h3lis331dl)
    {
        SensorConfig config{"H3LIS331DL", Config::Sensors::H3LIS331DL::RATE,
                        [this]() { h3lis331dlCallback(); }, 0};
        map.emplace(h3lis331dl.get(), config);
    }

    if (lis2mdl_ext)
    {
        SensorConfig config{"LIS2MDL_EXT", Config::Sensors::LIS2MDL::RATE,
                        [this]() { lis2mdlExtCallback(); }, 0};
        map.emplace(lis2mdl_ext.get(), config);
    }

    if (lis2mdl)
    {
        SensorConfig config{"LIS2MDL", Config::Sensors::LIS2MDL::RATE,
                        [this]() { lis2mdlCallback(); }, 0};
        map.emplace(lis2mdl.get(), config);
    }

    if (ubxgps)
    {
        SensorConfig config{"UBXGPS", Config::Sensors::UBXGPS::RATE,
                        [this]() { ubxgpsCallback(); }, 0};
        map.emplace(ubxgps.get(), config);
    }

    if (lsm6dsrx_0)
    {
        SensorConfig config{"LSM6DSRX_0", Config::Sensors::LSM6DSRX_0::RATE,
                        [this]() { lsm6dsrx0Callback(); }, 0};
        map.emplace(lsm6dsrx_0.get(), config);
    }

    if (lsm6dsrx_1)
    {
        SensorConfig config{"LSM6DSRX_1", Config::Sensors::LSM6DSRX_1::RATE,
                        [this]() { lsm6dsrx1Callback(); }, 0};
        map.emplace(lsm6dsrx_1.get(), config);
    }

    if (vn100)
    {
        SensorConfig config{"VN100", Config::Sensors::VN100::RATE,
                        [this]() { vn100Callback(); }, 0};
        map.emplace(vn100.get(), config);
    }

    if (internalAdc)
    {
        SensorConfig config{"InternalADC", Config::Sensors::InternalADC::RATE,
                        [this]() { internalAdcCallback(); }, 0};
        map.emplace(internalAdc.get(), config);
    }

    if (nd015a_0)
    {
        SensorConfig config{"ND015A_0", Config::Sensors::ND015A::RATE,
                        [this]() { nd015a0Callback(); }, 0};
        map.emplace(nd015a_0.get(), config);
    }

    if (nd015a_1)
    {
        SensorConfig config{"ND015A_1", Config::Sensors::ND015A::RATE,
                        [this]() { nd015a1Callback(); }, 0};
        map.emplace(nd015a_1.get(), config);
    }

    if (nd015a_2)
    {
        SensorConfig config{"ND015A_2", Config::Sensors::ND015A::RATE,
                        [this]() { nd015a2Callback(); }, 0};
        map.emplace(nd015a_2.get(), config);
    }

    if (nd015a_3)
    {
        SensorConfig config{"ND015A_3", Config::Sensors::ND015A::RATE,
                        [this]() { nd015a3Callback(); }, 0};
        map.emplace(nd015a_3.get(), config);
    }

    if (rotatedImu)
    {
        SensorConfig config{"RotatedIMU", Config::Sensors::IMU::RATE,
                        [this]() { rotatedImuCallback(); }, 0};
        map.emplace(rotatedImu.get(), config);
    }

    SensorManager::SchedulerMap_t schedulerMap = {std::pair<uint8_t, TaskScheduler*>(0,&getSensorsScheduler())};
    manager = std::make_unique<SensorManager>(map, schedulerMap);
    manager->startAll();
    return true;
}
