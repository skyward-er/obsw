/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

using namespace Main;
using namespace Boardcore;
using namespace miosix;
using namespace Eigen;

bool Sensors::isStarted() { return started; }

bool Sensors::start()
{
    // Read the magnetometer calibration from predefined file
    magCalibration.fromFile(Config::Sensors::MAG_CALIBRATION_FILENAME);

    if (Config::Sensors::LPS22DF::ENABLED && Config::Sensors::USING_LPS22DF)
        lps22dfInit();

    if (Config::Sensors::LPS28DFW::ENABLED)
        lps28dfwInit();

    if (Config::Sensors::H3LIS331DL::ENABLED)
        h3lis331dlInit();

    if (Config::Sensors::LIS2MDL::ENABLED && !Config::Sensors::USING_LPS22DF)
        lis2mdlExtInit();

    if (Config::Sensors::LIS2MDL::ENABLED)
        lis2mdlInExtInit();

    if (Config::Sensors::UBXGPS::ENABLED)
        ubxgpsInit();

    if (Config::Sensors::LSM6DSRX_0::ENABLED)
        lsm6dsrx0Init();

    if (Config::Sensors::LSM6DSRX_1::ENABLED)
        lsm6dsrx1Init();

    if (Config::Sensors::VN100::ENABLED)
        vn100Init();

    if (Config::Sensors::ADS131M08::ENABLED)
        ads131m08Init();

    if (Config::Sensors::ND015A_0::ENABLED)
        nd015a0Init();

    if (Config::Sensors::ND015A_1::ENABLED)
        nd015a1Init();

    if (Config::Sensors::ND015A_2::ENABLED)
        nd015a2Init();

    if (Config::Sensors::ND015A_3::ENABLED)
        nd015a3Init();

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

void Sensors::calibrate()
{
    BiasCalibration gyroCalibrator{};
    float staticPressure1Acc = 0.0f;
    float staticPressure2Acc = 0.0f;
    float dplBayPressureAcc  = 0.0f;
    float lps28dfwAcc        = 0.0f;

    for (int i = 0; i < Config::Sensors::CALIBRATION_SAMPLES_COUNT; i++)
    {
        auto lsm6dsrx        = getLSM6DSRXLastSample();
        auto staticPressure1 = getStaticPressure1LastSample();
        auto staticPressure2 = getStaticPressure2LastSample();
        auto dplBayPressure  = getDplBayPressureLastSample();
        auto lps28dfw        = getLPS28DFWLastSample();

        gyroCalibrator.feed(static_cast<GyroscopeData>(lsm6dsrx));
        staticPressure1Acc += staticPressure1.pressure;
        staticPressure2Acc += staticPressure2.pressure;
        dplBayPressureAcc += dplBayPressure.pressure;
        lps28dfwAcc += lps28dfw.pressure;

        Thread::sleep(Config::Sensors::CALIBRATION_SLEEP_TIME);
    }

    staticPressure1Acc /= Config::Sensors::CALIBRATION_SAMPLES_COUNT;
    staticPressure2Acc /= Config::Sensors::CALIBRATION_SAMPLES_COUNT;
    dplBayPressureAcc /= Config::Sensors::CALIBRATION_SAMPLES_COUNT;
    lps28dfwAcc /= Config::Sensors::CALIBRATION_SAMPLES_COUNT;

    // Calibrate all analog pressure sensors against the LPS28DFW or the
    // telemetry reference
    float reference;
    {
        Lock<FastMutex> lock{baroCalibrationMutex};
        reference = useBaroCalibrationReference ? baroCalibrationReference
                                                : lps28dfwAcc;
    }

    if (reference > Config::Sensors::ATMOS_CALIBRATION_THRESH)
    {
        // Calibrate sensors only if reference is valid
        // LPS28DFW might be disabled or unresponsive
        staticPressure1->updateOffset(staticPressure1Acc - reference);
        staticPressure2->updateOffset(staticPressure2Acc - reference);
        dplBayPressure->updateOffset(dplBayPressureAcc - reference);
    }

    {
        Lock<FastMutex> lock{gyroCalibrationMutex};
        gyroCalibration = gyroCalibrator.computeResult();
    }

    CalibrationData calibration = getCalibration();
    sdLogger.log(calibration);
}

CalibrationData Sensors::getCalibration()
{
    Lock<FastMutex> lock1{magCalibrationMutex};
    Lock<FastMutex> lock2{gyroCalibrationMutex};

    CalibrationData data;
    data.timestamp = TimestampTimer::getTimestamp();

    data.gyroBiasX         = gyroCalibration.getb().x();
    data.gyroBiasY         = gyroCalibration.getb().y();
    data.gyroBiasZ         = gyroCalibration.getb().z();
    data.magBiasX          = magCalibration.getb().x();
    data.magBiasY          = magCalibration.getb().y();
    data.magBiasZ          = magCalibration.getb().z();
    data.magScaleX         = magCalibration.getA().x();
    data.magScaleY         = magCalibration.getA().y();
    data.magScaleZ         = magCalibration.getA().z();
    data.staticPress1Bias  = staticPressure1->getOffset();
    data.staticPress1Scale = 1.0f;
    data.staticPress2Bias  = staticPressure2->getOffset();
    data.staticPress2Scale = 1.0f;
    data.dplBayPressBias   = dplBayPressure->getOffset();
    data.dplBayPressScale  = 1.0f;

    return data;
}

void Sensors::setBaroCalibrationReference(float reference)
{
    Lock<FastMutex> lock{baroCalibrationMutex};
    baroCalibrationReference    = reference;
    useBaroCalibrationReference = true;
}

void Sensors::resetBaroCalibrationReference()
{
    Lock<FastMutex> lock{baroCalibrationMutex};
    baroCalibrationReference    = 0.0f;
    useBaroCalibrationReference = false;
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

LPS28DFWData Sensors::getLPS28DFWLastSample()
{
    return lps28dfw ? lps28dfw->getLastSample() : LPS28DFWData{};
}

H3LIS331DLData Sensors::getH3LIS331DLLastSample()
{
    return h3lis331dl ? h3lis331dl->getLastSample() : H3LIS331DLData{};
}

LIS2MDLData Sensors::getLIS2MDLExtLastSample()
{
    return lis2mdl_ext ? lis2mdl_ext->getLastSample() : LIS2MDLData{};
}

LIS2MDLData Sensors::getLIS2MDLInExtLastSample()
{
    return lis2mdl_in_ext ? lis2mdl_in_ext->getLastSample() : LIS2MDLData{};
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

ADS131M08Data Sensors::getADS131M08LastSample()
{
    return ads131m08 ? ads131m08->getLastSample() : ADS131M08Data{};
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

ND015XData Sensors::getND015A0LastSample()
{
    return nd015a_1 ? nd015a_1->getLastSample() : ND015XData{};
}

ND015XData Sensors::getND015A0LastSample()
{
    return nd015a_3 ? nd015a_2->getLastSample() : ND015XData{};
}

ND015XData Sensors::getND015A0LastSample()
{
    return nd015a_3 ? nd015a_3->getLastSample() : ND015XData{};
}

LIS2MDLData Sensors::getCalibratedLIS2MDLLastSample()
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

LSM6DSRXData Sensors::getCalibratedLSM6DSRXLastSample()
{
    auto sample = getLSM6DSRXLastSample();

    {
        // This is for my boy Giuseppe <3
        Lock<FastMutex> lock{gyroCalibrationMutex};
        auto corrected =
            gyroCalibration.correct(static_cast<GyroscopeData>(sample));
        sample.angularSpeedX = corrected.x();
        sample.angularSpeedY = corrected.y();
        sample.angularSpeedZ = corrected.z();
    }

    return sample;
}

IMUData Sensors::getIMULastSample()
{
    return rotatedImu ? rotatedImu->getLastSample() : IMUData{};
}

PressureData Sensors::getAtmosPressureLastSample()
{
    if (Config::Sensors::Atmos::USE_PORT_2)
        return getStaticPressure2LastSample();
    else
        return getStaticPressure1LastSample();
}

PressureData Sensors::getCanPitotDynamicPressLastSample()
{
    Lock<FastMutex> lock{canMutex};
    return canPitotDynamicPressure;
}

PressureData Sensors::getCanPitotStaticPressLastSample()
{
    Lock<FastMutex> lock{canMutex};
    return canPitotStaticPressure;
}

PressureData Sensors::getCanTopTankPressLastSample()
{
    Lock<FastMutex> lock{canMutex};
    return canTopTankPressure;
}

PressureData Sensors::getCanBottomTankPressLastSample()
{
    Lock<FastMutex> lock{canMutex};
    return canBottomTankPressure;
}

PressureData Sensors::getCanCCPressLastSample()
{
    Lock<FastMutex> lock{canMutex};
    return canCCPressure;
}

TemperatureData Sensors::getCanTankTempLastSample()
{
    Lock<FastMutex> lock{canMutex};
    return canTankTemperature;
}

VoltageData Sensors::getCanMotorBatteryVoltageLastSample()
{
    Lock<FastMutex> lock{canMutex};
    return canMotorBatteryVoltage;
}

void Sensors::setCanPitotDynamicPress(PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canPitotDynamicPressure = data;
}

void Sensors::setCanPitotStaticPress(PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canPitotStaticPressure = data;
}

void Sensors::setCanTopTankPress(PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canTopTankPressure = data;
}

void Sensors::setCanBottomTankPress(PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canBottomTankPressure = data;
}

void Sensors::setCanCCPress(PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canCCPressure = data;
}

void Sensors::setCanTankTemp(TemperatureData data)
{
    Lock<FastMutex> lock{canMutex};
    canTankTemperature = data;
}

void Sensors::setCanMotorBatteryVoltage(VoltageData data)
{
    Lock<FastMutex> lock{canMutex};
    canMotorBatteryVoltage = data;
}

std::vector<SensorInfo> Sensors::getSensorInfos()
{
    if (manager)
    {
        std::vector<SensorInfo> infos{};

        if (lps22df)
            infos.push_back(manager->getSensorInfo(lps22df.get()));

        if (lps28dfw)
            infos.push_back(manager->getSensorInfo(lps28dfw.get()));

        if (h3lis331dl)
            infos.push_back(manager->getSensorInfo(h3lis331dl.get()));

        if (lis2mdl_ext)
            infos.push_back(manager->getSensorInfo(lis2mdl_ext.get()));

        if (lis2mdl_in_ext)
            infos.push_back(manager->getSensorInfo(lis2mdl_in_ext.get()));

        if (ubxgps)
            infos.push_back(manager->getSensorInfo(ubxgps.get()));

        if (lsm6dsrx_0)
            infos.push_back(manager->getSensorInfo(lsm6dsrx_0.get()));

        if (lsm6dsrx_1)
            infos.push_back(manager->getSensorInfo(lsm6dsrx_1.get()));

        if (vn100)
            infos.push_back(manager->getSensorInfo(vn100.get()));

        if (ads131m08)
            infos.push_back(manager->getSensorInfo(ads131m08.get()));

        if (internalAdc)
            infos.push_back(manager->getSensorInfo(internalAdc.get()));

        if (nd015a_0)
            infos.push_back(manager->getSensorInfo(nd015a_0.get()));

        if (nd015a_1)
            infos.push_back(manager->getSensorInfo(nd015a_1.get()));

        if (nd015a_2)
            infos.push_back(manager->getSensorInfo(nd015a_2.get()));

        if (nd015a_3)
            infos.push_back(manager->getSensorInfo(nd015a_3.get()));

        if (rotatedImu)
            infos.push_back(manager->getSensorInfo(rotatedImu.get()));

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

void Sensors::lps28dfwInit()
{
    LPS28DFW::SensorConfig config;
    config.sa0  = true;
    config.fsr  = Config::Sensors::LPS28DFW::FS;
    config.avg  = Config::Sensors::LPS28DFW::AVG;
    config.odr  = Config::Sensors::LPS28DFW::ODR;
    config.drdy = false;

    lps28dfw =
        std::make_unique<LPS28DFW>(getModule<Buses>()->getLPS28DFW(), config);
}

void Sensors::lps28dfwCallback() { sdLogger.log(getLPS28DFWLastSample()); }

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
    SPIBusConfig spiConfig = H3LIS331DL::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    LIS2MDL::Config config;
    config.deviceMode         = LIS2MDL::MD_CONTINUOUS;
    config.odr                = Config::Sensors::LIS2MDL::ODR;
    config.temperatureDivider = Config::Sensors::LIS2MDL::TEMP_DIVIDER;

    lis2mdl_ext = std::make_unique<LIS2MDL>(getModule<Buses>()->getLIS2MDL(),
                                            sensors::LIS2MDL_EXT::cs::getPin(),
                                            spiConfig, config);
}

void Sensors::lis2mdlExtCallback() { sdLogger.log(getLIS2MDLExtLastSample()); }

void Sensors::lis2mdlInExtInit()
{
    SPIBusConfig spiConfig = H3LIS331DL::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    LIS2MDL::Config config;
    config.deviceMode         = LIS2MDL::MD_CONTINUOUS;
    config.odr                = Config::Sensors::LIS2MDL::ODR;
    config.temperatureDivider = Config::Sensors::LIS2MDL::TEMP_DIVIDER;

    lis2mdl_in_ext = std::make_unique<LIS2MDL>(
        getModule<Buses>()->getLIS2MDL(), sensors::LIS2MDL_IN_EXT::cs::getPin(),
        spiConfig, config);
}

void Sensors::lis2mdlInExtCallback()
{
    sdLogger.log(getLIS2MDLInExtLastSample());
}

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
    config.fifoTemperatureBdr = LSM6DSRXConfig::FIFO_TEMPERATURE_BDR::DISABLED;

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
        sdLogger.log(lastFifo.at(i));
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
    config.fifoTemperatureBdr = LSM6DSRXConfig::FIFO_TEMPERATURE_BDR::DISABLED;

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
        sdLogger.log(lastFifo.at(i));
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

void Sensors::ads131m08Init()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    ADS131M08::Config config;
    config.oversamplingRatio = Config::Sensors::ADS131M08::OSR;
    config.globalChopModeEnabled =
        Config::Sensors::ADS131M08::GLOBAL_CHOP_MODE_EN;

    // Disable all channels
    config.channelsConfig[0].enabled = false;
    config.channelsConfig[1].enabled = false;
    config.channelsConfig[2].enabled = false;
    config.channelsConfig[3].enabled = false;
    config.channelsConfig[4].enabled = false;
    config.channelsConfig[5].enabled = false;
    config.channelsConfig[6].enabled = false;
    config.channelsConfig[7].enabled = false;

    // Enable required channels
    config.channelsConfig[(
        int)Config::Sensors::ADS131M08::STATIC_PRESSURE_1_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(
        int)Config::Sensors::ADS131M08::STATIC_PRESSURE_2_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(
        int)Config::Sensors::ADS131M08::DPL_BAY_PRESSURE_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    ads131m08 = std::make_unique<ADS131M08>(getModule<Buses>()->getADS131M08(),
                                            sensors::ADS131M08::cs::getPin(),
                                            spiConfig, config);
}

void Sensors::ads131m08Callback() { sdLogger.log(getADS131M08LastSample()); }

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
        spiConfig, Config::Sensors::ND015A_0::IOW,
        Config::Sensors::ND015A_0::BWL, Config::Sensors::ND015A_0::NTC,
        Config::Sensors::ND015A_0::ODR);
}

void Sensors::nd015a0Callback() { sdLogger.log(getND015A0LastSample()); }

void Sensors::nd015a1Init()
{
    SPIBusConfig spiConfig = ND015A::getDefaultSPIConfig();

    nd015a_1 = std::make_unique<ND015A>(
        getModule<Buses>()->getND015A(), sensors::ND015A_1::cs::getPin(),
        spiConfig, Config::Sensors::ND015A_1::IOW,
        Config::Sensors::ND015A_1::BWL, Config::Sensors::ND015A_1::NTC,
        Config::Sensors::ND015A_1::ODR);
}

void Sensors::nd015a1Callback() { sdLogger.log(getND015A1LastSample()); }

void Sensors::nd015a2Init()
{
    SPIBusConfig spiConfig = ND015A::getDefaultSPIConfig();

    nd015a_2 = std::make_unique<ND015A>(
        getModule<Buses>()->getND015A(), sensors::ND015A_2::cs::getPin(),
        spiConfig, Config::Sensors::ND015A_2::IOW,
        Config::Sensors::ND015A_2::BWL, Config::Sensors::ND015A_2::NTC,
        Config::Sensors::ND015A_2::ODR);
}

void Sensors::nd015a2Callback() { sdLogger.log(getND015A2LastSample()); }

void Sensors::nd015a3Init()
{
    SPIBusConfig spiConfig = ND015A::getDefaultSPIConfig();

    nd015a_3 = std::make_unique<ND015A>(
        getModule<Buses>()->getND015A(), sensors::ND015A_3::cs::getPin(),
        spiConfig, Config::Sensors::ND015A_3::IOW,
        Config::Sensors::ND015A_3::BWL, Config::Sensors::ND015A_3::NTC,
        Config::Sensors::ND015A_3::ODR);
}

void Sensors::nd015a3Callback() { sdLogger.log(getND015A3LastSample()); }

void Sensors::rotatedImuInit()
{
    rotatedImu = std::make_unique<RotatedIMU>(
        [this]()
        {
            auto imu6 = Config::Sensors::IMU::USE_CALIBRATED_LSM6DSRX
                            ? getCalibratedLSM6DSRXLastSample()
                            : getLSM6DSRXLastSample();
            auto mag  = Config::Sensors::IMU::USE_CALIBRATED_LIS2MDL
                            ? getCalibratedLIS2MDLLastSample()
                            : getLIS2MDLInExtLastSample();

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
        SensorInfo info{"LPS22DF", Config::Sensors::LPS22DF::RATE,
                        [this]() { lps22dfCallback(); }};
        map.emplace(lps22df.get(), info);
    }

    if (lps28dfw)
    {
        SensorInfo info{"LPS28DFW", Config::Sensors::LPS28DFW::RATE,
                        [this]() { lps28dfwCallback(); }};
        map.emplace(lps28dfw.get(), info);
    }

    if (h3lis331dl)
    {
        SensorInfo info{"H3LIS331DL", Config::Sensors::H3LIS331DL::RATE,
                        [this]() { h3lis331dlCallback(); }};
        map.emplace(h3lis331dl.get(), info);
    }

    if (lis2mdl_ext)
    {
        SensorInfo info{"LIS2MDL", Config::Sensors::LIS2MDL::RATE,
                        [this]() { lis2mdlExtCallback(); }};
        map.emplace(lis2mdl_ext.get(), info);
    }

    if (lis2mdl_in_ext)
    {
        SensorInfo info{"LIS2MDL", Config::Sensors::LIS2MDL::RATE,
                        [this]() { lis2mdlInExtCallback(); }};
        map.emplace(lis2mdl_in_ext.get(), info);
    }

    if (ubxgps)
    {
        SensorInfo info{"UBXGPS", Config::Sensors::UBXGPS::RATE,
                        [this]() { ubxgpsCallback(); }};
        map.emplace(ubxgps.get(), info);
    }

    if (lsm6dsrx_0)
    {
        SensorInfo info{"LSM6DSRX", Config::Sensors::LSM6DSRX_0::RATE,
                        [this]() { lsm6dsrxCallback(); }};
        map.emplace(lsm6dsrx_0.get(), info);
    }

    if (lsm6dsrx_1)
    {
        SensorInfo info{"LSM6DSRX", Config::Sensors::LSM6DSRX_1::RATE,
                        [this]() { lsm6dsrxCallback(); }};
        map.emplace(lsm6dsrx_1.get(), info);
    }

    if (vn100)
    {
        SensorInfo info{"VN100", Config::Sensors::VN100::RATE,
                        [this]() { vn100Callback(); }};
        map.emplace(vn100.get(), info);
    }

    if (ads131m08)
    {
        SensorInfo info{"ADS131M08", Config::Sensors::ADS131M08::RATE,
                        [this]() { ads131m08Callback(); }};
        map.emplace(ads131m08.get(), info);
    }

    if (internalAdc)
    {
        SensorInfo info{"InternalADC", Config::Sensors::InternalADC::RATE,
                        [this]() { internalAdcCallback(); }};
        map.emplace(internalAdc.get(), info);
    }

    if (nd015a_0)
    {
        SensorInfo info{"ND015A0", Config::Sensors::ND015A_0::RATE,
                        [this]() { nd015a0Callback(); }};
        map.emplace(nd015a_0.get(), info);
    }

    if (nd015a_1)
    {
        SensorInfo info{"ND015A1", Config::Sensors::ND015A_1::RATE,
                        [this]() { nd015a1Callback(); }};
        map.emplace(nd015a_1.get(), info);
    }

    if (nd015a_2)
    {
        SensorInfo info{"ND015A2", Config::Sensors::ND015A_2::RATE,
                        [this]() { nd015a2Callback(); }};
        map.emplace(nd015a_2.get(), info);
    }

    if (nd015a_3)
    {
        SensorInfo info{"ND015A3", Config::Sensors::ND015A_3::RATE,
                        [this]() { nd015a3Callback(); }};
        map.emplace(nd015a_3.get(), info);
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
