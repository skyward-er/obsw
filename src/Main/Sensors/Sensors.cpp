/* Copyright (c) 2024-2026 Skyward Experimental Rocketry
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

    accCalibrationLow.fromFile(
        Config::Sensors::LSM6DSRX_LOW::ACC_CALIBRATION_FILENAME);
    gyroCalibrationLow.fromFile(
        Config::Sensors::LSM6DSRX_LOW::GYRO_CALIBRATION_FILENAME);

    accCalibrationHigh.fromFile(
        Config::Sensors::LSM6DSRX_HIGH::ACC_CALIBRATION_FILENAME);
    gyroCalibrationHigh.fromFile(
        Config::Sensors::LSM6DSRX_HIGH::GYRO_CALIBRATION_FILENAME);

    accVN100Calibration.fromFile(
        Config::Sensors::VN100::ACC_CALIBRATION_FILENAME);

    gyroVN100Calibration.fromFile(
        Config::Sensors::VN100::GYRO_CALIBRATION_FILENAME);

    if (Config::Sensors::AS5047D_LEFT::ENABLED)
        as5047dLeftInit();

    if (Config::Sensors::AS5047D_RIGHT::ENABLED)
        as5047dRightInit();
    if (Config::Sensors::LPS22DF::ENABLED)
        lps22dfInit();

    if (Config::Sensors::H3LIS331DL::ENABLED)
        h3lis331dlInit();

    if (Config::Sensors::LIS2MDL_RCS::ENABLED)
        lis2mdlRcsInit();

    if (Config::Sensors::LIS2MDL_INT::ENABLED)
        lis2mdlIntInit();

    if (Config::Sensors::UBXGPS::ENABLED)
        ubxgpsInit();

    if (Config::Sensors::LSM6DSRX_LOW::ENABLED)
        lsm6dsrxLowInit();

    if (Config::Sensors::LSM6DSRX_HIGH::ENABLED)
        lsm6dsrxHighInit();

    if (Config::Sensors::VN100::ENABLED)
        vn100Init();

    if (Config::Sensors::ND015A::ENABLED)
    {
        nd015a0Init();
        nd015a1Init();
        nd015a2Init();
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
            auto mag = getLIS2MDLRcsLastSample();

            std::lock_guard<std::mutex> lock{magCalibrationMutex};
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

void Sensors::calibrate()
{
    // Log the current calibration
    sdLogger.log(getCalibration());
}

CalibrationData Sensors::getCalibration()
{
    std::lock(magCalibrationMutex, lsm6CalibrationLowMutex,
              lsm6CalibrationHighMutex, vn100CalibrationMutex);
    std::lock_guard<std::mutex> magLk(magCalibrationMutex, std::adopt_lock);
    std::lock_guard<std::mutex> lsm0lk(lsm6CalibrationLowMutex,
                                       std::adopt_lock);
    std::lock_guard<std::mutex> lsm1lk(lsm6CalibrationHighMutex,
                                       std::adopt_lock);
    std::lock_guard<std::mutex> vn100lk(vn100CalibrationMutex, std::adopt_lock);

    auto accBiasLow    = accCalibrationLow.getV();
    auto gyroBiasLow   = gyroCalibrationLow.getV();
    auto accBiasHigh   = accCalibrationHigh.getV();
    auto gyroBiasHigh  = gyroCalibrationHigh.getV();
    auto vn100AccBias  = accVN100Calibration.getV();
    auto vn100GyroBias = gyroVN100Calibration.getV();
    auto magBias       = magCalibration.getb();
    auto magScale      = magCalibration.getA();

    return {
        .timestamp      = TimestampTimer::getTimestamp(),
        .accLowBiasX    = accBiasLow.x(),
        .accLowBiasY    = accBiasLow.y(),
        .accLowBiasZ    = accBiasLow.z(),
        .gyroLowBiasX   = gyroBiasLow.x(),
        .gyroLowBiasY   = gyroBiasLow.y(),
        .gyroLowBiasZ   = gyroBiasLow.z(),
        .accHighBiasX   = accBiasHigh.x(),
        .accHighBiasY   = accBiasHigh.y(),
        .accHighBiasZ   = accBiasHigh.z(),
        .gyroHighBiasX  = gyroBiasHigh.x(),
        .gyroHighBiasY  = gyroBiasHigh.y(),
        .gyroHighBiasZ  = gyroBiasHigh.z(),
        .vn100AccBiasX  = vn100AccBias.x(),
        .vn100AccBiasY  = vn100AccBias.y(),
        .vn100AccBiasZ  = vn100AccBias.z(),
        .vn100GyroBiasX = vn100GyroBias.x(),
        .vn100GyroBiasY = vn100GyroBias.y(),
        .vn100GyroBiasZ = vn100GyroBias.z(),
        .magBiasX       = magBias.x(),
        .magBiasY       = magBias.y(),
        .magBiasZ       = magBias.z(),
        .magScaleX      = magScale.x(),
        .magScaleY      = magScale.y(),
        .magScaleZ      = magScale.z(),
    };
}

void Sensors::resetMagCalibrator()
{
    std::lock_guard<std::mutex> lock{magCalibrationMutex};
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
    std::lock_guard<std::mutex> lock{magCalibrationMutex};

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

Boardcore::AS5047DData Sensors::getAS5047DLeftLastSample()
{
    return as5047d_left ? as5047d_left->getLastSample() : AS5047DData{};
}

Boardcore::AS5047DData Sensors::getAS5047DRightLastSample()
{
    return as5047d_right ? as5047d_right->getLastSample() : AS5047DData{};
}

LPS22DFData Sensors::getLPS22DFLastSample()
{
    return lps22df ? lps22df->getLastSample() : LPS22DFData{};
}

H3LIS331DLData Sensors::getH3LIS331DLLastSample()
{
    return h3lis331dl ? h3lis331dl->getLastSample() : H3LIS331DLData{};
}

LIS2MDLData Sensors::getLIS2MDLRcsLastSample()
{
    return lis2mdl_rcs ? lis2mdl_rcs->getLastSample() : LIS2MDLData{};
}

LIS2MDLData Sensors::getLIS2MDLIntLastSample()
{
    return lis2mdl_int ? lis2mdl_int->getLastSample() : LIS2MDLData{};
}

UBXGPSData Sensors::getUBXGPSLastSample()
{
    return ubxgps ? ubxgps->getLastSample() : UBXGPSData{};
}

LSM6DSRXData Sensors::getLSM6DSRXLowLastSample()
{
    return lsm6dsrx_low ? lsm6dsrx_low->getLastSample() : LSM6DSRXData{};
}

LSM6DSRXData Sensors::getLSM6DSRXHighLastSample()
{
    return lsm6dsrx_high ? lsm6dsrx_high->getLastSample() : LSM6DSRXData{};
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
    return nd015a_2 ? nd015a_2->getLastSample() : ND015XData{};
}

LIS2MDLData Sensors::getCalibratedLIS2MDLRcsLastSample()
{
    auto sample = getLIS2MDLRcsLastSample();

    {
        std::lock_guard<std::mutex> lock{magCalibrationMutex};
        auto corrected =
            magCalibration.correct(static_cast<MagnetometerData>(sample));
        sample.magneticFieldX = corrected.x();
        sample.magneticFieldY = corrected.y();
        sample.magneticFieldZ = corrected.z();
    }

    return sample;
}

LIS2MDLData Sensors::getCalibratedLIS2MDLIntLastSample()
{
    auto sample = getLIS2MDLIntLastSample();

    {
        std::lock_guard<std::mutex> lock{magCalibrationMutex};
        auto corrected =
            magCalibration.correct(static_cast<MagnetometerData>(sample));
        sample.magneticFieldX = corrected.x();
        sample.magneticFieldY = corrected.y();
        sample.magneticFieldZ = corrected.z();
    }

    return sample;
}

LSM6DSRXData Sensors::getCalibratedLSM6DSRXLowLastSample()
{
    auto sample = getLSM6DSRXLowLastSample();
    std::lock_guard<std::mutex> lock{lsm6CalibrationLowMutex};

    auto correctedAcc =
        accCalibrationLow.correct(static_cast<AccelerometerData>(sample));
    sample.accelerationX = correctedAcc.x();
    sample.accelerationY = correctedAcc.y();
    sample.accelerationZ = correctedAcc.z();

    auto correctedGyro =
        gyroCalibrationLow.correct(static_cast<GyroscopeData>(sample));
    sample.angularSpeedX = correctedGyro.x();
    sample.angularSpeedY = correctedGyro.y();
    sample.angularSpeedZ = correctedGyro.z();

    return sample;
}

LSM6DSRXData Sensors::getCalibratedLSM6DSRXHighLastSample()
{
    auto sample = getLSM6DSRXHighLastSample();
    std::lock_guard<std::mutex> lock{lsm6CalibrationHighMutex};

    auto correctedAcc =
        accCalibrationHigh.correct(static_cast<AccelerometerData>(sample));
    sample.accelerationX = correctedAcc.x();
    sample.accelerationY = correctedAcc.y();
    sample.accelerationZ = correctedAcc.z();

    auto correctedGyro =
        gyroCalibrationHigh.correct(static_cast<GyroscopeData>(sample));
    sample.angularSpeedX = correctedGyro.x();
    sample.angularSpeedY = correctedGyro.y();
    sample.angularSpeedZ = correctedGyro.z();

    return sample;
}

VN100SpiData Sensors::getCalibratedVN100LastSample()
{
    auto sample = getVN100LastSample();
    std::lock_guard<std::mutex> lock{vn100CalibrationMutex};

    auto correctedAcc =
        accVN100Calibration.correct(static_cast<AccelerometerData>(sample));
    sample.accelerationX = correctedAcc.x();
    sample.accelerationY = correctedAcc.y();
    sample.accelerationZ = correctedAcc.z();

    auto correctedGyro =
        gyroVN100Calibration.correct(static_cast<GyroscopeData>(sample));
    sample.angularSpeedX = correctedGyro.x();
    sample.angularSpeedY = correctedGyro.y();
    sample.angularSpeedZ = correctedGyro.z();

    return sample;
}

IMUData Sensors::getIMULastSample()
{
    return rotatedImu ? rotatedImu->getLastSample() : IMUData{};
}

PressureData Sensors::getAtmosPressureLastSample()
{
    auto atmosPressure = PressureData{};

    atmosPressureFilter.add(getND015A0LastSample().pressure);
    atmosPressureFilter.add(getND015A1LastSample().pressure);
    atmosPressureFilter.add(getND015A2LastSample().pressure);

    atmosPressure.pressureTimestamp = getND015A0LastSample().pressureTimestamp;
    atmosPressure.pressure          = atmosPressureFilter.calcMedian();

    return atmosPressure;
}

TemperatureData Sensors::getTemperatureLastSample()
{
    return getLSM6DSRXLowLastSample();
}

PressureData Sensors::getCanPitotTotalPressure()
{
    std::lock_guard<std::mutex> lock{canMutex};
    return canPitotTotalPressure;
}

PressureData Sensors::getCanPitotStaticPressure()
{
    std::lock_guard<std::mutex> lock{canMutex};
    return canPitotStaticPressure;
}

void Sensors::setCanPitotTotalPressure(PressureData data)
{
    std::lock_guard<std::mutex> lock{canMutex};
    canPitotTotalPressure = data;
}

PressureData Sensors::getCanPitotDynamicPressure()
{
    std::lock_guard<std::mutex> lock{canMutex};
    return PressureData{
        .pressureTimestamp = canPitotTotalPressure.pressureTimestamp,
        .pressure =
            canPitotTotalPressure.pressure - canPitotStaticPressure.pressure,
    };
}

void Sensors::setCanPitotStaticPressure(PressureData data)
{
    std::lock_guard<std::mutex> lock{canMutex};
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
        infos.push_back(SensorInfo{name, 0, nullptr, false})

        PUSH_SENSOR_INFO(as5047d_left, "AS5047D_LEFT");
        PUSH_SENSOR_INFO(as5047d_right, "AS5047D_RIGHT");
        PUSH_SENSOR_INFO(lis2mdl_rcs, "LIS2MDL_RCS");
        PUSH_SENSOR_INFO(lps22df, "LPS22DF");
        PUSH_SENSOR_INFO(h3lis331dl, "H3LIS331DL");
        PUSH_SENSOR_INFO(vn100, "VN100");
        PUSH_SENSOR_INFO(ubxgps, "UBXGPS");
        PUSH_SENSOR_INFO(lis2mdl_int, "LIS2MDL_INT");
        PUSH_SENSOR_INFO(lsm6dsrx_low, "LSM6DSRX_LOW");
        PUSH_SENSOR_INFO(lsm6dsrx_high, "LSM6DSRX_HIGH");
        PUSH_SENSOR_INFO(nd015a_0, "ND015A_0");
        PUSH_SENSOR_INFO(nd015a_1, "ND015A_1");
        PUSH_SENSOR_INFO(nd015a_2, "ND015A_2");
        PUSH_SENSOR_INFO(internalAdc, "InternalADC");
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

void ::Sensors::as5047dLeftInit()
{
    SPIBusConfig spiConfig = AS5047DSPI::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    AS5047DSPIConfig config;
    config.daecEnabled = Config::Sensors::AS5047D_LEFT::DAEC_EN;
    config.dataType    = Config::Sensors::AS5047D_LEFT::DATA_SELECT;
    config.rotationDirection =
        Config::Sensors::AS5047D_LEFT::ROTATION_DIRECTION;

    as5047d_left = std::make_unique<AS5047DSPI>(
        getModule<Buses>()->getAS5047DLeft(), sensors::AS5047D_1::cs::getPin(),
        spiConfig, config);
}

void Sensors::as5047dLeftCallback()
{
    sdLogger.log(AS5047DLeftData(getAS5047DLeftLastSample()));
}

void Sensors::as5047dRightInit()
{
    SPIBusConfig spiConfig = AS5047DSPI::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    AS5047DSPIConfig config;
    config.daecEnabled = Config::Sensors::AS5047D_RIGHT::DAEC_EN;
    config.dataType    = Config::Sensors::AS5047D_RIGHT::DATA_SELECT;
    config.rotationDirection =
        Config::Sensors::AS5047D_RIGHT::ROTATION_DIRECTION;

    as5047d_right = std::make_unique<AS5047DSPI>(
        getModule<Buses>()->getAS5047DRight(), sensors::AS5047D_0::cs::getPin(),
        spiConfig, config);
}

void Sensors::as5047dRightCallback()
{
    sdLogger.log(AS5047DRightData(getAS5047DRightLastSample()));
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

void Sensors::lis2mdlRcsInit()
{
    SPIBusConfig spiConfig = LIS2MDL::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    LIS2MDL::Config config;
    config.deviceMode         = LIS2MDL::MD_CONTINUOUS;
    config.odr                = Config::Sensors::LIS2MDL_INT::ODR;
    config.temperatureDivider = Config::Sensors::LIS2MDL_INT::TEMP_DIVIDER;

    lis2mdl_rcs = std::make_unique<LIS2MDL>(getModule<Buses>()->getLIS2MDLRcs(),
                                            sensors::LIS2MDL_RCS::cs::getPin(),
                                            spiConfig, config);
}

void Sensors::lis2mdlRcsCallback()
{
    sdLogger.log(LIS2MDLExternalData{getLIS2MDLRcsLastSample()});
}

void Sensors::lis2mdlIntInit()
{
    SPIBusConfig spiConfig = LIS2MDL::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    LIS2MDL::Config config;
    config.deviceMode         = LIS2MDL::MD_CONTINUOUS;
    config.odr                = Config::Sensors::LIS2MDL_RCS::ODR;
    config.temperatureDivider = Config::Sensors::LIS2MDL_RCS::TEMP_DIVIDER;

    lis2mdl_int = std::make_unique<LIS2MDL>(getModule<Buses>()->getLIS2MDLInt(),
                                            sensors::LIS2MDL_INT::cs::getPin(),
                                            spiConfig, config);
}

void Sensors::lis2mdlIntCallback() { sdLogger.log(getLIS2MDLIntLastSample()); }

void Sensors::ubxgpsInit()
{
    SPIBusConfig spiConfig = UBXGPSSpi::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;

    ubxgps = std::make_unique<UBXGPSSpi>(getModule<Buses>()->getUBXGps(),
                                         sensors::UBXGps::cs::getPin(),
                                         spiConfig, 5);
}

void Sensors::ubxgpsCallback() { sdLogger.log(getUBXGPSLastSample()); }

void Sensors::lsm6dsrxLowInit()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;
    spiConfig.mode         = SPI::Mode::MODE_0;

    LSM6DSRXConfig config;
    config.bdu = LSM6DSRXConfig::BDU::CONTINUOUS_UPDATE;

    config.fsAcc     = Config::Sensors::LSM6DSRX_LOW::ACC_FS;
    config.odrAcc    = Config::Sensors::LSM6DSRX_LOW::ACC_ODR;
    config.opModeAcc = Config::Sensors::LSM6DSRX_LOW::ACC_OP_MODE;

    config.fsGyr     = Config::Sensors::LSM6DSRX_LOW::GYR_FS;
    config.odrGyr    = Config::Sensors::LSM6DSRX_LOW::GYR_ODR;
    config.opModeGyr = Config::Sensors::LSM6DSRX_LOW::GYR_OP_MODE;

    config.fifoMode = LSM6DSRXConfig::FIFO_MODE::CONTINUOUS;
    config.fifoTimestampDecimation =
        LSM6DSRXConfig::FIFO_TIMESTAMP_DECIMATION::DEC_1;
    config.fifoTemperatureBdr = LSM6DSRXConfig::FIFO_TEMPERATURE_BDR::HZ_52;

    lsm6dsrx_low = std::make_unique<LSM6DSRX>(
        getModule<Buses>()->getLSM6DSRX(), sensors::LSM6DSRX_LOW::cs::getPin(),
        spiConfig, config);
}

void Sensors::lsm6dsrxLowCallback()
{
    if (!lsm6dsrx_low)
        return;

    // For every instance inside the fifo log the sample
    uint16_t lastFifoSize;
    const auto lastFifo = lsm6dsrx_low->getLastFifo(lastFifoSize);
    for (uint16_t i = 0; i < lastFifoSize; i++)
        sdLogger.log(LSM6DSRX0Data{lastFifo.at(i)});
}

void Sensors::lsm6dsrxHighInit()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;
    spiConfig.mode         = SPI::Mode::MODE_0;

    LSM6DSRXConfig config;
    config.bdu = LSM6DSRXConfig::BDU::CONTINUOUS_UPDATE;

    config.fsAcc     = Config::Sensors::LSM6DSRX_HIGH::ACC_FS;
    config.odrAcc    = Config::Sensors::LSM6DSRX_HIGH::ACC_ODR;
    config.opModeAcc = Config::Sensors::LSM6DSRX_HIGH::ACC_OP_MODE;

    config.fsGyr     = Config::Sensors::LSM6DSRX_HIGH::GYR_FS;
    config.odrGyr    = Config::Sensors::LSM6DSRX_HIGH::GYR_ODR;
    config.opModeGyr = Config::Sensors::LSM6DSRX_HIGH::GYR_OP_MODE;

    config.fifoMode = LSM6DSRXConfig::FIFO_MODE::CONTINUOUS;
    config.fifoTimestampDecimation =
        LSM6DSRXConfig::FIFO_TIMESTAMP_DECIMATION::DEC_1;
    config.fifoTemperatureBdr = LSM6DSRXConfig::FIFO_TEMPERATURE_BDR::HZ_52;

    lsm6dsrx_high = std::make_unique<LSM6DSRX>(
        getModule<Buses>()->getLSM6DSRX(), sensors::LSM6DSRX_HIGH::cs::getPin(),
        spiConfig, config);
}

void Sensors::lsm6dsrxHighCallback()
{
    if (!lsm6dsrx_high)
        return;

    // For every instance inside the fifo log the sample
    uint16_t lastFifoSize;
    const auto lastFifo = lsm6dsrx_high->getLastFifo(lastFifoSize);
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
                                       400);
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

/**
 * Sets the ascent phase for double LSM6DSRX sensor management
 * @param isAscent True if the rocket is in ascent phase, false otherwise.
 */
void Sensors::setAscentPhase(bool isAscent) { ascentPhase = isAscent; }

void Sensors::rotatedImuInit()
{
    rotatedImu = std::make_unique<RotatedIMU>(
        [this]()
        {
    #if defined(DUAL_LSM6)  // Dual LSM6 sensor for ascent & descent phases
            Boardcore::LSM6DSRXData lsmData;

            if (ascentPhase)
            {
                // HIGH Gs
                lsmData = Config::Sensors::IMU::USE_CALIBRATED_LSM6DSRX
                              ? getCalibratedLSM6DSRXHighLastSample()
                              : getLSM6DSRXHighLastSample();
            }
            else
            {
                // LOW Gs
                lsmData = Config::Sensors::IMU::USE_CALIBRATED_LSM6DSRX
                              ? getCalibratedLSM6DSRXLowLastSample()
                              : getLSM6DSRXLowLastSample();
            }

            return IMUData{lsmData, lsmData, mag};

    #else   // Main VN100 sensor
        auto mag = Config::Sensors::IMU::USE_CALIBRATED_LIS2MDL
                                ? getCalibratedLIS2MDLRcsLastSample()
                                : getLIS2MDLRcsLastSample();

                    auto vnData = Config::Sensors::IMU::USE_CALIBRATED_VN100
                                    ? getCalibratedVN100LastSample()
                                    : getVN100LastSample();

                    Boardcore::AccelerometerData acc(
                        vnData.accelerationTimestamp, vnData.accelerationX,
                        vnData.accelerationY, vnData.accelerationZ);
                    Boardcore::GyroscopeData gyro(
                        vnData.angularSpeedTimestamp, vnData.angularSpeedX,
                        vnData.angularSpeedY, vnData.angularSpeedZ);

                    return IMUData{acc, gyro, mag};
                
    #endif
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

    if (as5047d_left)
    {
        SensorInfo info{"AS5047D_LEFT", Config::Sensors::AS5047D_LEFT::RATE,
                        [this]() { as5047dLeftCallback(); }};
        map.emplace(as5047d_left.get(), info);
    }

    if (as5047d_right)
    {
        SensorInfo info{"AS5047D_RIGHT", Config::Sensors::AS5047D_RIGHT::RATE,
                        [this]() { as5047dRightCallback(); }};
        map.emplace(as5047d_right.get(), info);
    }

    if (lis2mdl_rcs)
    {
        SensorInfo info{"LIS2MDL_RCS", Config::Sensors::LIS2MDL_RCS::RATE,
                        [this]() { lis2mdlRcsCallback(); }};
        map.emplace(lis2mdl_rcs.get(), info);
    }

    if (lps22df)
    {
        SensorInfo info{"LPS22DF", Config::Sensors::LPS22DF::RATE,
                        [this]() { lps22dfCallback(); }};
        map.emplace(lps22df.get(), info);
    }

    if (h3lis331dl)
    {
        SensorInfo info{"H3LIS331DL", Config::Sensors::H3LIS331DL::RATE,
                        [this]() { h3lis331dlCallback(); }};
        map.emplace(h3lis331dl.get(), info);
    }

    if (vn100)
    {
        SensorInfo info{"VN100", Config::Sensors::VN100::RATE,
                        [this]() { vn100Callback(); }};
        map.emplace(vn100.get(), info);
    }

    if (ubxgps)
    {
        SensorInfo info{"UBXGPS", Config::Sensors::UBXGPS::RATE,
                        [this]() { ubxgpsCallback(); }};
        map.emplace(ubxgps.get(), info);
    }

    if (lis2mdl_int)
    {
        SensorInfo info{"LIS2MDL_INT", Config::Sensors::LIS2MDL_INT::RATE,
                        [this]() { lis2mdlIntCallback(); }};
        map.emplace(lis2mdl_int.get(), info);
    }

    if (lsm6dsrx_low)
    {
        SensorInfo info{"LSM6DSRX_LOW", Config::Sensors::LSM6DSRX_LOW::RATE,
                        [this]() { lsm6dsrxLowCallback(); }};
        map.emplace(lsm6dsrx_low.get(), info);
    }

    if (lsm6dsrx_high)
    {
        SensorInfo info{"LSM6DSRX_HIGH", Config::Sensors::LSM6DSRX_HIGH::RATE,
                        [this]() { lsm6dsrxHighCallback(); }};
        map.emplace(lsm6dsrx_high.get(), info);
    }

    if (internalAdc)
    {
        SensorInfo info{"InternalADC", Config::Sensors::InternalADC::RATE,
                        [this]() { internalAdcCallback(); }};
        map.emplace(internalAdc.get(), info);
    }

    if (nd015a_0)
    {
        SensorInfo info{"ND015A_0", Config::Sensors::ND015A::RATE,
                        [this]() { nd015a0Callback(); }};
        map.emplace(nd015a_0.get(), info);
    }

    if (nd015a_1)
    {
        SensorInfo info{"ND015A_1", Config::Sensors::ND015A::RATE,
                        [this]() { nd015a1Callback(); }};
        map.emplace(nd015a_1.get(), info);
    }

    if (nd015a_2)
    {
        SensorInfo info{"ND015A_2", Config::Sensors::ND015A::RATE,
                        [this]() { nd015a2Callback(); }};
        map.emplace(nd015a_2.get(), info);
    }

    if (rotatedImu)
    {
        SensorInfo info{"RotatedIMU", Config::Sensors::IMU::RATE,
                        [this]() { rotatedImuCallback(); }};
        map.emplace(rotatedImu.get(), info);
    }

    manager = std::make_unique<SensorManager>(map, &getSensorsScheduler());
    return manager->start();
}
