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

#include <utils/ModuleManager/ModuleManager.hpp>

#include "SensorsData.h"

using namespace Main;
using namespace Boardcore;
using namespace miosix;

bool Sensors::isStarted() { return started; }

bool Sensors::start()
{
    // Read the magnetometer calibration from predefined file
    magCalibration.fromFile("/sd/magCalibration.csv");

    if (Config::Sensors::LPS22DF::ENABLED)
    {
        lps22dfInit();
    }

    if (Config::Sensors::LPS28DFW::ENABLED)
    {
        lps28dfwInit();
    }

    if (Config::Sensors::H3LIS331DL::ENABLED)
    {
        h3lis331dlInit();
    }

    if (Config::Sensors::LIS2MDL::ENABLED)
    {
        lis2mdlInit();
    }

    if (Config::Sensors::UBXGPS::ENABLED)
    {
        ubxgpsInit();
    }

    if (Config::Sensors::LSM6DSRX::ENABLED)
    {
        lsm6dsrxInit();
    }

    if (Config::Sensors::ADS131M08::ENABLED)
    {
        ads131m08Init();
        staticPressure1Init();
        staticPressure2Init();
        dplBayPressureInit();
    }

    if (Config::Sensors::InternalADC::ENABLED)
    {
        internalAdcInit();
    }

    if (Config::Sensors::RotatedIMU::ENABLED)
    {
        rotatedIMUInit();
    }

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

    started = true;
    return true;
}

void Sensors::calibrate()
{
    // Create the stats to calibrate the barometers
    Stats staticPressure1Stats;
    Stats staticPressure2Stats;
    Stats deploymentPressureStats;
    Stats gyroStats;

    // Add N samples to the stats
    for (unsigned int i = 0; i < Config::Sensors::CALIBRATION_SAMPLES; i++)
    {
        staticPressure1Stats.add(getStaticPressure1().pressure);
        staticPressure2Stats.add(getStaticPressure2().pressure);
        deploymentPressureStats.add(getDplBayPressure().pressure);

        // Calibrate gyroscope

        // Delay for the expected period
        miosix::Thread::sleep(Config::Sensors::CALIBRATION_PERIOD);
    }

    // Compute the difference between the mean value from LPS28DFW
    float refPressure = reference.refPressure;

    staticPressure1->updateOffset(staticPressure1Stats.getStats().mean -
                                  refPressure);
    staticPressure2->updateOffset(staticPressure2Stats.getStats().mean -
                                  refPressure);
    dplBayPressure->updateOffset(deploymentPressureStats.getStats().mean -
                                 refPressure);

    // Log the offsets
    SensorsCalibrationParameter cal{};
    cal.timestamp     = TimestampTimer::getTimestamp();
    cal.offsetStatic1 = staticPressure1Stats.getStats().mean - refPressure;
    cal.offsetStatic2 = staticPressure2Stats.getStats().mean - refPressure;
    cal.offsetDeployment =
        deploymentPressureStats.getStats().mean - refPressure;
    cal.referencePressure = refPressure;

    Logger::getInstance().log(cal);
}

bool Sensors::writeMagCalibration()
{
    // Compute the calibration result in protected area
    {
        miosix::Lock<FastMutex> l(calibrationMutex);
        SixParametersCorrector cal = magCalibrator.computeResult();

        // Check result validity
        if (!std::isnan(cal.getb()[0]) && !std::isnan(cal.getb()[1]) &&
            !std::isnan(cal.getb()[2]) && !std::isnan(cal.getA()[0]) &&
            !std::isnan(cal.getA()[1]) && !std::isnan(cal.getA()[2]))
        {
            magCalibration = cal;

            // Save the calibration to the calibration file
            return magCalibration.toFile("/sd/magCalibration.csv");
        }
        return false;
    }
}

Boardcore::LPS22DFData Sensors::getLPS22DFLastSample()
{
    return lps22df ? lps22df->getLastSample() : LPS22DFData{};
}

Boardcore::LPS28DFWData Sensors::getLPS28DFWLastSample()
{
    return lps28dfw ? lps28dfw->getLastSample() : LPS28DFWData{};
}

Boardcore::H3LIS331DLData Sensors::getH3LIS331DLLastSample()
{
    return h3lis331dl ? h3lis331dl->getLastSample() : H3LIS331DLData{};
}

Boardcore::LIS2MDLData Sensors::getLIS2MDLLastSample()
{
    return lis2mdl ? lis2mdl->getLastSample() : LIS2MDLData{};
}

Boardcore::UBXGPSData Sensors::getUBXGPSLastSample()
{
    return ubxgps ? ubxgps->getLastSample() : UBXGPSData{};
}

Boardcore::LSM6DSRXData Sensors::getLSM6DSRXLastSample()
{
    return lsm6dsrx ? lsm6dsrx->getLastSample() : LSM6DSRXData{};
}

Boardcore::ADS131M08Data Sensors::getADS131M08LastSample()
{
    return ads131m08 ? ads131m08->getLastSample() : ADS131M08Data{};
}

Boardcore::InternalADCData Sensors::getInternalADCLastSample()
{
    return internalAdc ? internalAdc->getLastSample() : InternalADCData{};
}
RotatedIMUData Sensors::getRotatedIMULastSample()
{
    return imu ? imu->getLastSample() : RotatedIMUData{};
}

Boardcore::VoltageData Sensors::getBatteryVoltage()
{
    auto sample   = getInternalADCLastSample();
    float voltage = sample.voltage[(int)Config::Sensors::InternalADC::VBAT_CH] *
                    Config::Sensors::InternalADC::VBAT_SCALE;
    return {sample.timestamp, voltage};
}

Boardcore::VoltageData Sensors::getCamBatteryVoltage()
{
    auto sample = getInternalADCLastSample();
    float voltage =
        sample.voltage[(int)Config::Sensors::InternalADC::CAM_VBAT_CH] *
        Config::Sensors::InternalADC::CAM_VBAT_SCALE;
    return {sample.timestamp, voltage};
}

MagnetometerData Sensors::getCalibratedMagnetometerLastSample()
{
    // Do not need to pause the kernel, the last sample getter is already
    // protected
    MagnetometerData lastSample = getLIS2MDLLastSample();
    MagnetometerData result;

    // Correct the result and copy the timestamp
    {
        miosix::Lock<FastMutex> l(calibrationMutex);
        result =
            static_cast<MagnetometerData>(magCalibration.correct(lastSample));
    }

    result.magneticFieldTimestamp = lastSample.magneticFieldTimestamp;
    return result;
}

PressureData Sensors::getStaticPressure1()
{
    return staticPressure1 ? static_cast<Boardcore::PressureData>(
                                 staticPressure1->getLastSample())
                           : PressureData{};
}

PressureData Sensors::getStaticPressure2()
{
    return staticPressure2 ? static_cast<Boardcore::PressureData>(
                                 staticPressure2->getLastSample())
                           : PressureData{};
}

PressureData Sensors::getDplBayPressure()
{
    return dplBayPressure ? static_cast<Boardcore::PressureData>(
                                dplBayPressure->getLastSample())
                          : PressureData{};
}

PressureData Sensors::getCanTopTankPress1()
{
    Lock<FastMutex> lock{canMutex};
    return canTopTankPressure1;
}

PressureData Sensors::getCanTopTankPress2()
{
    Lock<FastMutex> lock{canMutex};
    return canTopTankPressure2;
}

PressureData Sensors::getCanCCPress()
{
    Lock<FastMutex> lock{canMutex};
    return canCCPressure;
}

TemperatureData Sensors::getCanTankTemp()
{
    Lock<FastMutex> lock{canMutex};
    return canTankTemperature;
}

VoltageData Sensors::getCanMotorBatteryVoltage()
{
    Lock<FastMutex> lock{canMutex};
    return canMotorBatteryVoltage;
}

void Sensors::setCanTopTankPress1(Boardcore::PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canTopTankPressure1 = data;
}

void Sensors::setCanTopTankPress2(Boardcore::PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canTopTankPressure2 = data;
}

void Sensors::setCanCCPress(Boardcore::PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canCCPressure = data;
}

void Sensors::setCanTankTemp(Boardcore::TemperatureData data)
{
    Lock<FastMutex> lock{canMutex};
    canTankTemperature = data;
}

void Sensors::setCanMotorBatteryVoltage(Boardcore::VoltageData data)
{
    Lock<FastMutex> lock{canMutex};
    canMotorBatteryVoltage = data;
}

std::vector<Boardcore::SensorInfo> Sensors::getSensorInfos()
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
                manager->getSensorInfo(internalAdc.get()),
                manager->getSensorInfo(staticPressure1.get()),
                manager->getSensorInfo(staticPressure2.get()),
                manager->getSensorInfo(dplBayPressure.get()),
                manager->getSensorInfo(imu.get())};
    }
    else
    {
        return {};
    }
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

void Sensors::lps22dfCallback()
{
    Logger::getInstance().log(getLPS22DFLastSample());
}

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

void Sensors::lps28dfwCallback()
{
    Logger::getInstance().log(getLPS28DFWLastSample());
}

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
    Logger::getInstance().log(getH3LIS331DLLastSample());
}

void Sensors::lis2mdlInit()
{
    SPIBusConfig spiConfig = H3LIS331DL::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    LIS2MDL::Config config;
    config.deviceMode         = LIS2MDL::MD_CONTINUOUS;
    config.odr                = Config::Sensors::LIS2MDL::ODR;
    config.temperatureDivider = Config::Sensors::LIS2MDL::TEMP_DIVIDER;

    lis2mdl = std::make_unique<LIS2MDL>(getModule<Buses>()->getLIS2MDL(),
                                        sensors::LIS2MDL::cs::getPin(),
                                        spiConfig, config);
}

void Sensors::lis2mdlCallback()
{
    Logger::getInstance().log(getLIS2MDLLastSample());
}

void Sensors::ubxgpsInit()
{
    SPIBusConfig spiConfig = UBXGPSSpi::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;

    ubxgps = std::make_unique<UBXGPSSpi>(getModule<Buses>()->getUBXGps(),
                                         sensors::UBXGps::cs::getPin(),
                                         spiConfig, 5);
}

void Sensors::ubxgpsCallback()
{
    Logger::getInstance().log(getUBXGPSLastSample());
}

void Sensors::lsm6dsrxInit()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;
    spiConfig.mode         = SPI::Mode::MODE_0;

    LSM6DSRXConfig config;
    config.bdu = LSM6DSRXConfig::BDU::CONTINUOUS_UPDATE;

    config.fsAcc     = Config::Sensors::LSM6DSRX::ACC_FS;
    config.odrAcc    = Config::Sensors::LSM6DSRX::ACC_ODR;
    config.opModeAcc = Config::Sensors::LSM6DSRX::ACC_OP_MODE;

    config.fsGyr     = Config::Sensors::LSM6DSRX::GYR_FS;
    config.odrGyr    = Config::Sensors::LSM6DSRX::GYR_ODR;
    config.opModeGyr = Config::Sensors::LSM6DSRX::GYR_OP_MODE;

    config.fifoMode = LSM6DSRXConfig::FIFO_MODE::CONTINUOUS;
    config.fifoTimestampDecimation =
        LSM6DSRXConfig::FIFO_TIMESTAMP_DECIMATION::DEC_1;
    config.fifoTemperatureBdr = LSM6DSRXConfig::FIFO_TEMPERATURE_BDR::DISABLED;

    lsm6dsrx = std::make_unique<LSM6DSRX>(getModule<Buses>()->getLSM6DSRX(),
                                          sensors::LSM6DSRX::cs::getPin(),
                                          spiConfig, config);
}

void Sensors::lsm6dsrxCallback()
{
    Logger::getInstance().log(getLSM6DSRXLastSample());
}

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
    config.channelsConfig[0].enabled = true;
    config.channelsConfig[1].enabled = true;
    config.channelsConfig[2].enabled = true;

    ads131m08 = std::make_unique<ADS131M08>(getModule<Buses>()->getADS131M08(),
                                            sensors::ADS131M08::cs::getPin(),
                                            spiConfig, config);
}

void Sensors::ads131m08Callback()
{
    Logger::getInstance().log(getADS131M08LastSample());
}

void Sensors::internalAdcInit()
{
    internalAdc = std::make_unique<InternalADC>(ADC2);
    internalAdc->enableChannel(Config::Sensors::InternalADC::VBAT_CH);
    internalAdc->enableChannel(Config::Sensors::InternalADC::CAM_VBAT_CH);
    internalAdc->enableChannel(Config::Sensors::InternalADC::CUTTER_SENSE_CH);
    internalAdc->enableTemperature();
    internalAdc->enableVbat();
}

void Sensors::internalAdcCallback()
{
    Logger::getInstance().log(getInternalADCLastSample());
}

void Sensors::staticPressure1Init()
{
    staticPressure1 = std::make_unique<MPXH6115A>(
        [this]()
        {
            auto sample  = getADS131M08LastSample();
            auto voltage = sample.getVoltage(
                Config::Sensors::ADS131M08::STATIC_PRESSURE_1_CHANNEL);
            voltage.voltage *=
                Config::Sensors::ADS131M08::STATIC_PRESSURE_1_SCALE;

            return voltage;
        });
}

void Sensors::staticPressure1Callback()
{
    Logger::getInstance().log(StaticPressureData1{getStaticPressure1()});
}

void Sensors::staticPressure2Init()
{
    staticPressure2 = std::make_unique<MPXH6115A>(
        [this]()
        {
            auto sample  = getADS131M08LastSample();
            auto voltage = sample.getVoltage(
                Config::Sensors::ADS131M08::STATIC_PRESSURE_2_CHANNEL);
            voltage.voltage *=
                Config::Sensors::ADS131M08::STATIC_PRESSURE_2_SCALE;

            return voltage;
        });
}

void Sensors::staticPressure2Callback()
{
    Logger::getInstance().log(StaticPressureData2{getStaticPressure2()});
}

void Sensors::dplBayPressureInit()
{
    dplBayPressure = std::make_unique<MPXH6115A>(
        [this]()
        {
            auto sample  = getADS131M08LastSample();
            auto voltage = sample.getVoltage(
                Config::Sensors::ADS131M08::DPL_BAY_PRESSURE_CHANNEL);
            voltage.voltage *=
                Config::Sensors::ADS131M08::DPL_BAY_PRESSURE_SCALE;

            return voltage;
        });
}

void Sensors::dplBayPressureCallback()
{
    Logger::getInstance().log(DplBayPressureData{getDplBayPressure()});
}

void Sensors::rotatedIMUInit()
{
    // Register the IMU as the fake sensor, passing as parameters the methods to
    // retrieve real data. The sensor is not synchronized, but the sampling
    // thread is always the same.
    imu = std::make_unique<RotatedIMU>(
        [this]() { return getLSM6DSRXLastSample(); },
        [this]() { return getCalibratedMagnetometerLastSample(); },
        [this]() { return getLSM6DSRXLastSample(); });

    // Invert the Y axis on the magnetometer
    Eigen::Matrix3f m{{1, 0, 0}, {0, -1, 0}, {0, 0, 1}};
    imu->addMagTransformation(m);
    imu->addMagTransformation(imu->rotateAroundY(-90));
    imu->addMagTransformation(imu->rotateAroundZ(90));

    // Accelerometer
    imu->addAccTransformation(imu->rotateAroundZ(-90));
    imu->addAccTransformation(imu->rotateAroundX(-90));

    // Gyroscope
    imu->addGyroTransformation(imu->rotateAroundZ(-90));
    imu->addGyroTransformation(imu->rotateAroundX(-90));
}

void Sensors::rotatedIMUCallback()
{
    Logger::getInstance().log(getRotatedIMULastSample());
}

bool Sensors::sensorManagerInit()
{
    TaskScheduler &scheduler =
        getModule<BoardScheduler>()->getSensorsScheduler();

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

    if (lis2mdl)
    {
        SensorInfo info{"LIS2MDL", Config::Sensors::LIS2MDL::RATE,
                        [this]() { lis2mdlCallback(); }};
        map.emplace(lis2mdl.get(), info);
    }

    if (ubxgps)
    {
        SensorInfo info{"UBXGPS", Config::Sensors::UBXGPS::RATE,
                        [this]() { ubxgpsCallback(); }};
        map.emplace(ubxgps.get(), info);
    }

    if (lsm6dsrx)
    {
        SensorInfo info{"LSM6DSRX", Config::Sensors::LSM6DSRX::RATE,
                        [this]() { lsm6dsrxCallback(); }};
        map.emplace(lsm6dsrx.get(), info);
    }

    if (ads131m08)
    {
        SensorInfo info{"ADS131M08", Config::Sensors::ADS131M08::RATE,
                        [this]() { ads131m08Callback(); }};
        map.emplace(ads131m08.get(), info);
    }

    if (staticPressure1)
    {
        SensorInfo info{"StaticPressure1", Config::Sensors::ADS131M08::RATE,
                        [this]() { staticPressure1Callback(); }};
        map.emplace(staticPressure1.get(), info);
    }

    if (staticPressure2)
    {
        SensorInfo info{"StaticPressure2", Config::Sensors::ADS131M08::RATE,
                        [this]() { staticPressure2Callback(); }};
        map.emplace(staticPressure2.get(), info);
    }

    if (dplBayPressure)
    {
        SensorInfo info{"DplBayPressure", Config::Sensors::ADS131M08::RATE,
                        [this]() { dplBayPressureCallback(); }};
        map.emplace(dplBayPressure.get(), info);
    }

    if (internalAdc)
    {
        SensorInfo info{"InternalADC", Config::Sensors::InternalADC::RATE,
                        [this]() { internalAdcCallback(); }};
        map.emplace(internalAdc.get(), info);
    }

    if (imu)
    {
        SensorInfo info{"RotatedIMU", Config::Sensors::RotatedIMU::RATE,
                        [this]() { rotatedIMUCallback(); }};
        map.emplace(imu.get(), info);
    }

    manager = std::make_unique<SensorManager>(map, &scheduler);
    return manager->start();
}
