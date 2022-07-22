/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Luca Erbetta, Luca Conterio, Matteo Pignataro
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
#include <common/events/Events.h>
#include <drivers/interrupt/external_interrupts.h>
#include <events/EventBroker.h>

using namespace std;
using namespace Boardcore;
using namespace Common;
using namespace Payload::SensorsConfig;

// BMX160 Watermark interrupt
void __attribute__((used)) EXTI5_IRQHandlerImpl()
{
    if (Payload::Sensors::getInstance().bmx160 != nullptr)
        Payload::Sensors::getInstance().bmx160->IRQupdateTimestamp(
            TimestampTimer::getTimestamp());
}

namespace Payload
{

bool Sensors::start()
{
    miosix::GpioPin interruptPin = miosix::sensors::bmx160::intr::getPin();
    enableExternalInterrupt(interruptPin.getPort(), interruptPin.getNumber(),
                            InterruptTrigger::FALLING_EDGE, 0);

    return sensorManager->start();
}

bool Sensors::isStarted() { return sensorManager->areAllSensorsInitialized(); }

BMX160Data Sensors::getBMX160LastSample()
{
    miosix::PauseKernelLock lock;
    return bmx160->getLastSample();
}

BMX160WithCorrectionData Sensors::getBMX160WithCorrectionLastSample()
{
    miosix::PauseKernelLock lock;
    return bmx160WithCorrection->getLastSample();
}

LIS3MDLData Sensors::getMagnetometerLIS3MDLLastSample()
{
    miosix::PauseKernelLock lock;
    return lis3mdl->getLastSample();
}

MS5803Data Sensors::getMS5803LastSample()
{
    miosix::PauseKernelLock lock;
    return ms5803->getLastSample();
}

UBXGPSData Sensors::getUbxGpsLastSample()
{
    miosix::PauseKernelLock lock;
    return ubxGps->getLastSample();
}

ADS1118Data Sensors::getADS1118LastSample()
{
    miosix::PauseKernelLock lock;
    return ads1118->getLastSample();
}

MPXHZ6130AData Sensors::getStaticPressureLastSample()
{
    miosix::PauseKernelLock lock;
    return staticPressure->getLastSample();
}

SSCDANN030PAAData Sensors::getDplPressureLastSample()
{
    miosix::PauseKernelLock lock;
    return dplPressure->getLastSample();
}

SSCDRRN015PDAData Sensors::getPitotPressureLastSample()
{
    miosix::PauseKernelLock lock;
    return pitotPressure->getLastSample();
}

void Sensors::calibrate()
{
    calibrating = true;

    ms5803Stats.reset();
    staticPressureStats.reset();
    dplPressureStats.reset();
    pitotPressureStats.reset();

    bmx160WithCorrection->startCalibration();

    Thread::sleep(CALIBRATION_DURATION);

    bmx160WithCorrection->stopCalibration();

    // Calibrate the analog pressure sensor to the digital one
    float ms5803Mean         = ms5803Stats.getStats().mean;
    float staticPressureMean = staticPressureStats.getStats().mean;
    float dplPressureMean    = dplPressureStats.getStats().mean;
    staticPressure->setOffset(staticPressureMean - ms5803Mean);
    dplPressure->setOffset(dplPressureMean - ms5803Mean);
    pitotPressure->setOffset(pitotPressureStats.getStats().mean);

    calibrating = true;
}

std::map<string, bool> Sensors::getSensorsState()
{
    std::map<string, bool> sensorsState;

    for (auto sensor : sensorsMap)
        sensorsState[sensor.second.id] =
            sensorManager->getSensorInfo(sensor.first).isInitialized;

    return sensorsState;
}

Sensors::Sensors()
{
    // Initialize all the sensors
    bmx160Init();
    bmx160WithCorrectionInit();
    lis3mdlInit();
    ms5803Init();
    ubxGpsInit();
    pitotPressureInit();
    ads1118Init();
    dplPressureInit();
    pitotPressureInit();

    // Create the sensor manager
    sensorManager = new SensorManager(sensorsMap);

    // Check if the essential sensors are initialized correctly
    if (sensorManager->getSensorInfo(bmx160).isInitialized)
        // && sensorManager->getSensorInfo(gps).isInitialized)
        EventBroker::getInstance().post(FMM_INIT_OK, TOPIC_FMM);
}

Sensors::~Sensors()
{
    delete bmx160;
    delete lis3mdl;
    delete ms5803;
    delete ubxGps;
    delete ads1118;
    delete staticPressure;
    delete dplPressure;
    delete pitotPressure;
}

void Sensors::bmx160Init()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    BMX160Config config;
    config.fifoMode      = BMX160Config::FifoMode::HEADER;
    config.fifoWatermark = IMU_BMX_FIFO_WATERMARK;
    config.fifoInterrupt = BMX160Config::FifoInterruptPin::PIN_INT1;

    config.temperatureDivider = IMU_BMX_TEMP_DIVIDER;

    config.accelerometerRange = IMU_BMX_ACC_FULLSCALE_ENUM;
    config.gyroscopeRange     = IMU_BMX_GYRO_FULLSCALE_ENUM;

    config.accelerometerDataRate = IMU_BMX_ACC_GYRO_ODR_ENUM;
    config.gyroscopeDataRate     = IMU_BMX_ACC_GYRO_ODR_ENUM;
    config.magnetometerRate      = IMU_BMX_MAG_ODR_ENUM;

    config.gyroscopeUnit = BMX160Config::GyroscopeMeasureUnit::RAD;

    bmx160 =
        new BMX160(Buses::getInstance().spi2,
                   miosix::sensors::bmx160::cs::getPin(), config, spiConfig);

    SensorInfo info("BMX160", IMU_BMX_SAMPLE_PERIOD,
                    bind(&Sensors::bmx160Callback, this));

    sensorsMap.emplace(make_pair(bmx160, info));

    LOG_INFO(logger, "BMX160 setup done!");
}

void Sensors::bmx160Callback()
{
    uint8_t fifoSize = bmx160->getLastFifoSize();
    auto& fifo       = bmx160->getLastFifo();

    Logger::getInstance().log(bmx160->getTemperature());

    for (uint8_t i = 0; i < fifoSize; i++)
        Logger::getInstance().log(fifo.at(i));

    Logger::getInstance().log(bmx160->getFifoStats());
}

void Sensors::bmx160WithCorrectionInit()
{
    // Read the correction parameters
    BMX160CorrectionParameters correctionParameters =
        BMX160WithCorrection::readCorrectionParametersFromFile(
            BMX160_CORRECTION_PARAMETERS_FILE);

    bmx160WithCorrection = new BMX160WithCorrection(
        bmx160, correctionParameters, BMX160_AXIS_ROTATION);

    SensorInfo info(
        "BMX160WithCorrection", IMU_BMX_SAMPLE_PERIOD,
        [&]()
        { Logger::getInstance().log(bmx160WithCorrection->getLastSample()); });

    sensorsMap.emplace(make_pair(bmx160WithCorrection, info));

    LOG_INFO(logger, "BMX160WithCorrection setup done!");
}

void Sensors::lis3mdlInit()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    LIS3MDL::Config config;
    config.odr                = MAG_LIS_ODR_ENUM;
    config.scale              = MAG_LIS_FULLSCALE;
    config.temperatureDivider = 1;

    lis3mdl =
        new LIS3MDL(Buses::getInstance().spi2,
                    miosix::sensors::lis3mdl::cs::getPin(), spiConfig, config);

    // Create the sensor info
    SensorInfo info("LIS3MDL", MAG_LIS_SAMPLE_PERIOD,
                    [&]()
                    { Logger::getInstance().log(lis3mdl->getLastSample()); });
    sensorsMap.emplace(make_pair(lis3mdl, info));

    LOG_INFO(logger, "LIS3MDL setup done!");
}

void Sensors::ms5803Init()
{
    SPIBusConfig spiConfig{};
    spiConfig.mode         = SPI::Mode::MODE_3;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    ms5803 = new MS5803(Buses::getInstance().spi2,
                        miosix::sensors::ms5803::cs::getPin(), spiConfig,
                        PRESS_DIGITAL_TEMP_DIVIDER);

    SensorInfo info(
        "MS5803", PRESS_DIGITAL_SAMPLE_PERIOD,
        [&]()
        {
            Logger::getInstance().log(ms5803->getLastSample());

            if (calibrating && ms5803->getLastSample().pressure != 0)
                ms5803Stats.add(ms5803->getLastSample().pressure);
        });

    sensorsMap.emplace(make_pair(ms5803, info));

    LOG_INFO(logger, "MS5803 setup done!");
}

void Sensors::ubxGpsInit()
{
    ubxGps = new UBXGPSSerial(GPS_BAUD_RATE, GPS_SAMPLE_RATE, 2, "gps", 9600);

    SensorInfo info("UBXGPS", GPS_SAMPLE_PERIOD,
                    [&]()
                    { Logger::getInstance().log(ubxGps->getLastSample()); });
    sensorsMap.emplace(make_pair(ubxGps, info));

    LOG_INFO(logger, "UbloxGPS setup done!");
}

void Sensors::ads1118Init()
{
    SPIBusConfig spiConfig = ADS1118::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;

    ADS1118::ADS1118Config config = ADS1118::ADS1118_DEFAULT_CONFIG;
    config.bits.mode              = ADS1118::ADS1118Mode::CONTINUOUS_CONV_MODE;

    ads1118 =
        new ADS1118(Buses::getInstance().spi2,
                    miosix::sensors::ads1118::cs::getPin(), config, spiConfig);

    ads1118->enableInput(ADC_CH_STATIC_PORT, ADC_DR_STATIC_PORT,
                         ADC_PGA_STATIC_PORT);
    ads1118->enableInput(ADC_CH_PITOT_PORT, ADC_DR_PITOT_PORT,
                         ADC_PGA_PITOT_PORT);
    ads1118->enableInput(ADC_CH_DPL_PORT, ADC_DR_DPL_PORT, ADC_PGA_DPL_PORT);
    ads1118->enableInput(ADC_CH_VREF, ADC_DR_VREF, ADC_PGA_VREF);

    SensorInfo info("ADS1118", ADC_ADS1118_SAMPLE_PERIOD,
                    [&]()
                    { Logger::getInstance().log(ads1118->getLastSample()); });
    sensorsMap.emplace(make_pair(ads1118, info));

    LOG_INFO(logger, "ADS1118 adc setup done!");
}

void Sensors::staticPressureInit()
{
    function<ADCData()> readVoltage(
        bind(&ADS1118::getVoltage, ads1118, ADC_CH_STATIC_PORT));

    staticPressure = new MPXHZ6130A(readVoltage, REFERENCE_VOLTAGE);

    SensorInfo info(
        "StaticPortsBarometer", STATIC_PRESS_SAMPLE_PERIOD,
        [&]()
        {
            Logger::getInstance().log(staticPressure->getLastSample());

            if (calibrating)
                staticPressureStats.add(
                    staticPressure->getLastSample().pressure);
        });

    sensorsMap.emplace(make_pair(staticPressure, info));

    LOG_INFO(logger, "Static pressure sensor setup done!");
}

void Sensors::dplPressureInit()
{
    function<ADCData()> readVoltage(
        bind(&ADS1118::getVoltage, ads1118, ADC_CH_DPL_PORT));

    dplPressure = new SSCDANN030PAA(readVoltage, REFERENCE_VOLTAGE);

    SensorInfo info(
        "DPL_PRESSURE", DPL_PRESS_SAMPLE_PERIOD,
        [&]()
        {
            Logger::getInstance().log(dplPressure->getLastSample());

            if (calibrating)
                dplPressureStats.add(dplPressure->getLastSample().pressure);
        });

    sensorsMap.emplace(make_pair(dplPressure, info));

    LOG_INFO(logger, "Deployment pressure sensor setup done!");
}

void Sensors::pitotPressureInit()
{
    // Create a function to read the analog voltage
    function<ADCData()> readVoltage(
        bind(&ADS1118::getVoltage, ads1118, ADC_CH_PITOT_PORT));

    // Setup the pitot sensor
    pitotPressure = new SSCDRRN015PDA(readVoltage, REFERENCE_VOLTAGE);

    // Create the sensor info
    SensorInfo info(
        "PitotBarometer", PITOT_PRESS_SAMPLE_PERIOD,
        [&]()
        {
            Logger::getInstance().log(pitotPressure->getLastSample());

            if (calibrating)
                pitotPressureStats.add(pitotPressure->getLastSample().pressure);
        });

    sensorsMap.emplace(make_pair(pitotPressure, info));

    LOG_INFO(logger, "Pitot differential pressure sensor setup done!");
}

}  // namespace Payload