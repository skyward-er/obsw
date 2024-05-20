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

HSCMRNN015PAData Sensors::getStaticPressureLastSample()
{
    miosix::PauseKernelLock lock;
    return staticPressure != nullptr ? staticPressure->getLastSample()
                                     : HSCMRNN015PAData{};
}

SSCMRNN030PAData Sensors::getDynamicPressureLastSample()
{
    miosix::PauseKernelLock lock;
    return dynamicPressure != nullptr ? dynamicPressure->getLastSample()
                                      : SSCMRNN030PAData{};
}
PitotData Sensors::getPitotLastSample()
{
    miosix::PauseKernelLock lock;
    return pitot != nullptr ? pitot->getLastSample() : PitotData{};
}

// Processed Getters
BatteryVoltageSensorData Sensors::getBatteryVoltageLastSample()
{
    // Do not need to pause the kernel, the last sample getter is already
    // protected
    ADS131M08Data sample = getADS131M08LastSample();
    BatteryVoltageSensorData data;

    // Populate the data
    data.voltageTimestamp = sample.timestamp;
    data.channelId        = static_cast<uint8_t>(BATTERY_VOLTAGE_CHANNEL);
    data.voltage          = sample.getVoltage(BATTERY_VOLTAGE_CHANNEL).voltage;
    data.batVoltage       = sample.getVoltage(BATTERY_VOLTAGE_CHANNEL).voltage *
                      BATTERY_VOLTAGE_CONVERSION_FACTOR;
    return data;
}

BatteryVoltageSensorData Sensors::getCamBatteryVoltageLastSample()
{
    // Do not need to pause the kernel, the last sample getter is already
    // protected
    ADS131M08Data sample = getADS131M08LastSample();
    BatteryVoltageSensorData data;

    // Populate the data
    data.voltageTimestamp = sample.timestamp;
    data.channelId        = static_cast<uint8_t>(CAM_BATTERY_VOLTAGE_CHANNEL);
    data.voltage    = sample.getVoltage(CAM_BATTERY_VOLTAGE_CHANNEL).voltage;
    data.batVoltage = sample.getVoltage(CAM_BATTERY_VOLTAGE_CHANNEL).voltage *
                      BATTERY_VOLTAGE_CONVERSION_FACTOR;
    return data;
}

CurrentData Sensors::getCurrentLastSample()
{
    // Do not need to pause the kernel, the last sample getter is already
    // protected
    ADS131M08Data sample = getADS131M08LastSample();
    CurrentData data;

    // Populate the data
    data.currentTimestamp = sample.timestamp;
    data.current =
        (sample.getVoltage(CURRENT_CHANNEL).voltage - CURRENT_OFFSET) *
        CURRENT_CONVERSION_FACTOR;
    return data;
}

RotatedIMUData Sensors::getIMULastSample()
{
    miosix::PauseKernelLock lock;
    return imu != nullptr ? imu->getLastSample() : RotatedIMUData{};
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

Sensors::Sensors(TaskScheduler* sched, Buses* buses)
    : scheduler{sched}, buses{buses}, sensorsId(0)
{
    // Init all the sensors
    lps22dfInit();
    lps28dfw_1Init();
    lps28dfw_2Init();
    h3lis331dlInit();
    lis2mdlInit();
    ubxgpsInit();
    lsm6dsrxInit();
    // ads131m08Init();
    staticPressureInit();
    dynamicPressureInit();
    pitotInit();
    imuInit();
}

bool Sensors::start()
{
    // Read the magnetometer calibration from predefined file
    magCalibration.fromFile("magCalibration.csv");

    if (lps22df)
    {
        registerSensor(lps22df, "LPS22DF", LPS22DF_PERIOD,
                       [this]() { this->lps22dfCallback(); });
        addInfoGetter(lps22df);
    }

    if (lps28dfw_1)
    {
        registerSensor(lps28dfw_1, "LPS28DFW_1", LPS28DFW_PERIOD,
                       [this]() { this->lps28dfw_1Callback(); });
        addInfoGetter(lps28dfw_1);
    }

    if (lps28dfw_2)
    {
        registerSensor(lps28dfw_2, "LPS28DFW_2", LPS28DFW_PERIOD,
                       [this]() { this->lps28dfw_2Callback(); });
        addInfoGetter(lps28dfw_2);
    }

    if (h3lis331dl)
    {
        registerSensor(h3lis331dl, "h3lis331dl", H3LIS331DL_PERIOD,
                       [this]() { this->h3lis331dlCallback(); });
        addInfoGetter(h3lis331dl);
    }

    if (lis2mdl)
    {
        registerSensor(lis2mdl, "lis2mdl", LIS2MDL_PERIOD,
                       [this]() { this->lis2mdlCallback(); });
        addInfoGetter(lis2mdl);
    }

    if (ubxgps)
    {
        registerSensor(ubxgps, "ubxgps", UBXGPS_PERIOD,
                       [this]() { this->ubxgpsCallback(); });
        addInfoGetter(ubxgps);
    }

    if (lsm6dsrx)
    {
        registerSensor(lsm6dsrx, "lsm6dsrx", LSM6DSRX_PERIOD,
                       [this]() { this->lsm6dsrxCallback(); });
        addInfoGetter(lsm6dsrx);
    }

    if (ads131m08)
    {
        registerSensor(ads131m08, "ads131m08", ADS131M08_PERIOD,
                       [this]() { this->ads131m08Callback(); });
        addInfoGetter(ads131m08);
    }

    if (staticPressure)
    {
        registerSensor(staticPressure, "staticPressure", ADS131M08_PERIOD,
                       [this]() { this->staticPressureCallback(); });
    }

    if (dynamicPressure)
    {
        registerSensor(dynamicPressure, "dynamicPressure", ADS131M08_PERIOD,
                       [this]() { this->dynamicPressureCallback(); });
    }

    if (pitot)
    {
        registerSensor(pitot, "pitot", ADS131M08_PERIOD,
                       [this]() { this->pitotCallback(); });
    }

    if (imu)
    {
        registerSensor(imu, "RotatedIMU", IMU_PERIOD,
                       [this]() { this->imuCallback(); });
    }

    // Add the magnetometer calibration to the scheduler
    size_t result = scheduler->addTask(
        [&]()
        {
            // Gather the last sample data
            MagnetometerData lastSample = getLIS2MDLLastSample();

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
    // Create the stats to calibrate the barometers
    Stats lps28dfw1Stats;
    Stats lps28dfw2Stats;
    Stats staticPressureStats;
    Stats dynamicPressureStats;

    // Add N samples to the stats
    for (unsigned int i = 0; i < SensorsConfig::CALIBRATION_SAMPLES; i++)
    {
        lps28dfw1Stats.add(getLPS28DFW_1LastSample().pressure);
        lps28dfw2Stats.add(getLPS28DFW_2LastSample().pressure);
        staticPressureStats.add(getStaticPressureLastSample().pressure);
        dynamicPressureStats.add(getDynamicPressureLastSample().pressure);

        // Delay for the expected period
        miosix::Thread::sleep(SensorsConfig::CALIBRATION_PERIOD);
    }

    // Compute the difference between the mean value from LPS28DFW
    float reference =
        (lps28dfw1Stats.getStats().mean + lps28dfw2Stats.getStats().mean) / 2.f;

    staticPressure->updateOffset(staticPressureStats.getStats().mean -
                                 reference);
    dynamicPressure->updateOffset(dynamicPressureStats.getStats().mean -
                                  reference);

    // Log the offsets
    SensorsCalibrationParameter cal{};
    cal.timestamp         = TimestampTimer::getTimestamp();
    cal.offsetStatic      = staticPressureStats.getStats().mean - reference;
    cal.offsetDynamic     = dynamicPressureStats.getStats().mean - reference;
    cal.referencePressure = reference;

    Logger::getInstance().log(cal);
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

void Sensors::lps22dfInit()
{
    // Get the correct SPI configuration
    SPIBusConfig config = LPS22DF::getDefaultSPIConfig();
    config.clockDivider = SPI::ClockDivider::DIV_16;

    // Configure the device
    LPS22DF::Config sensorConfig;
    sensorConfig.avg = LPS22DF_AVG;
    sensorConfig.odr = LPS22DF_ODR;

    // Create sensor instance with configured parameters
    lps22df = new LPS22DF(buses->spi3, miosix::sensors::LPS22DF::cs::getPin(),
                          config, sensorConfig);
}
void Sensors::lps28dfw_1Init()
{
    // Configure the sensor
    LPS28DFW::SensorConfig config{false, LPS28DFW_FSR, LPS28DFW_AVG,
                                  LPS28DFW_ODR, false};

    // Create sensor instance with configured parameters
    lps28dfw_1 = new LPS28DFW(buses->i2c1, config);
}
void Sensors::lps28dfw_2Init()
{
    // Configure the sensor
    LPS28DFW::SensorConfig config{true, LPS28DFW_FSR, LPS28DFW_AVG,
                                  LPS28DFW_ODR, false};

    // Create sensor instance with configured parameters
    lps28dfw_2 = new LPS28DFW(buses->i2c1, config);
}
void Sensors::h3lis331dlInit()
{
    // Get the correct SPI configuration
    SPIBusConfig config = H3LIS331DL::getDefaultSPIConfig();
    config.clockDivider = SPI::ClockDivider::DIV_16;

    // Create sensor instance with configured parameters
    h3lis331dl =
        new H3LIS331DL(buses->spi3, miosix::sensors::H3LIS331DL::cs::getPin(),
                       config, H3LIS331DL_ODR, H3LIS331DL_BDU, H3LIS331DL_FSR);
}
void Sensors::lis2mdlInit()
{
    // Get the correct SPI configuration
    SPIBusConfig config = LIS2MDL::getDefaultSPIConfig();
    config.clockDivider = SPI::ClockDivider::DIV_16;

    // Configure the sensor
    LIS2MDL::Config sensorConfig;
    sensorConfig.deviceMode         = LIS2MDL_OPERATIVE_MODE;
    sensorConfig.odr                = LIS2MDL_ODR;
    sensorConfig.temperatureDivider = LIS2MDL_TEMPERATURE_DIVIDER;

    // Create sensor instance with configured parameters
    lis2mdl = new LIS2MDL(buses->spi3, miosix::sensors::LIS2MDL::cs::getPin(),
                          config, sensorConfig);
}

void Sensors::ubxgpsInit()
{
    // Get the correct SPI configuration
    SPIBusConfig config = UBXGPSSpi::getDefaultSPIConfig();
    config.clockDivider = SPI::ClockDivider::DIV_64;

    // Create sensor instance with configured parameters
    ubxgps = new UBXGPSSpi(buses->spi4, miosix::sensors::GPS::cs::getPin(),
                           config, 5);
}

void Sensors::lsm6dsrxInit()
{
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
    lsm6dsrx =
        new LSM6DSRX(buses->spi1, miosix::sensors::LSM6DSRX::cs::getPin(),
                     config, sensorConfig);
}

void Sensors::ads131m08Init()
{
    // Configure the SPI
    SPIBusConfig config;
    config.clockDivider = SPI::ClockDivider::DIV_32;

    // Configure the device
    ADS131M08::Config sensorConfig;
    sensorConfig.oversamplingRatio     = ADS131M08_OVERSAMPLING_RATIO;
    sensorConfig.globalChopModeEnabled = ADS131M08_GLOBAL_CHOP_MODE;

    // Create the sensor instance with configured parameters
    ads131m08 =
        new ADS131M08(buses->spi4, miosix::sensors::ADS131::cs::getPin(),
                      config, sensorConfig);
}

void Sensors::staticPressureInit()
{
    // create lambda function to read the voltage
    auto readVoltage = (
        [&]() -> ADCData
        {
             ADS131M08Data sample = ads131m08->getLastSample();
           return sample.getVoltage(
               STATIC_PRESSURE_CHANNEL);
        });

    staticPressure = new HSCMRNN015PA(readVoltage, ADC_VOLTAGE_RANGE);
}

void Sensors::dynamicPressureInit()
{
    // create lambda function to read the voltage
    auto readVoltage = (
        [&]() -> ADCData
        {
             auto sample = ads131m08->getLastSample();
             return sample.getVoltage(
                DYNAMIC_PRESSURE_CHANNEL);
        });

    dynamicPressure = new SSCMRNN030PA(readVoltage, ADC_VOLTAGE_RANGE);
}

void Sensors::pitotInit()
{
    // create lambda function to read the pressure
    function<float()> getDynamicPressure(
        [&]() { return getDynamicPressureLastSample().pressure; });
    function<float()> getStaticPressure(
        [&]() { return getStaticPressureLastSample().pressure; });

    pitot = new Pitot(getDynamicPressure, getStaticPressure);
    pitot->setReferenceValues(Common::ReferenceConfig::defaultReferenceValues);
}

void Sensors::pitotSetReferenceAltitude(float altitude)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock l;

    ReferenceValues reference = pitot->getReferenceValues();
    reference.refAltitude     = altitude;
    pitot->setReferenceValues(reference);
}

void Sensors::pitotSetReferenceTemperature(float temperature)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock l;

    ReferenceValues reference = pitot->getReferenceValues();
    reference.refTemperature  = temperature + 273.15f;
    pitot->setReferenceValues(reference);
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

void Sensors::imuInit()
{
    // Register the IMU as the fake sensor, passing as parameters the
    // methods to retrieve real data. The sensor is not synchronized, but
    // the sampling thread is always the same.
    imu = new RotatedIMU(
        bind(&LSM6DSRX::getLastSample, lsm6dsrx),
        bind(&Sensors::getCalibratedMagnetometerLastSample, this),
        bind(&LSM6DSRX::getLastSample, lsm6dsrx));

    // Invert the Y axis on the magnetometer
    Eigen::Matrix3f m{{1, 0, 0}, {0, -1, 0}, {0, 0, 1}};
    imu->addMagTransformation(m);
}

void Sensors::lps22dfCallback()
{
    LPS22DFData lastSample = lps22df->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::lps28dfw_1Callback()
{
    LPS28DFW_1Data lastSample =
        static_cast<LPS28DFW_1Data>(lps28dfw_1->getLastSample());

    Logger::getInstance().log(lastSample);
}
void Sensors::lps28dfw_2Callback()
{
    LPS28DFW_2Data lastSample =
        static_cast<LPS28DFW_2Data>(lps28dfw_2->getLastSample());
    Logger::getInstance().log(lastSample);
}
void Sensors::h3lis331dlCallback()
{
    H3LIS331DLData lastSample = h3lis331dl->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::lis2mdlCallback()
{
    LIS2MDLData lastSample = lis2mdl->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::ubxgpsCallback()
{
    UBXGPSData lastSample = ubxgps->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::lsm6dsrxCallback()
{
    LSM6DSRXData lastSample = lsm6dsrx->getLastSample();

    auto& fifo        = lsm6dsrx->getLastFifo();
    uint16_t fifoSize = lsm6dsrx->getLastFifoSize();

    // For every instance inside the fifo log the sample
    for (uint16_t i = 0; i < fifoSize; i++)
    {
        Logger::getInstance().log(fifo.at(i));
    }
    Logger::getInstance().log(lastSample);
}
void Sensors::ads131m08Callback()
{
    ADS131M08Data lastSample = ads131m08->getLastSample();
    Logger::getInstance().log(lastSample);
}

void Sensors::staticPressureCallback()
{
    HSCMRNN015PAData lastSample = staticPressure->getLastSample();
    Logger::getInstance().log(lastSample);
}

void Sensors::dynamicPressureCallback()
{
    SSCMRNN030PAData lastSample = dynamicPressure->getLastSample();
    Logger::getInstance().log(lastSample);
}

void Sensors::pitotCallback()
{
    PitotData lastSample = pitot->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::imuCallback()
{
    RotatedIMUData lastSample = imu->getLastSample();
    Logger::getInstance().log(lastSample);
}

}  // namespace Payload
