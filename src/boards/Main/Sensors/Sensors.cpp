/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Conterio, Alberto Nidasio
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

#include <Main/Buses.h>
#include <Main/Configs/SensorsConfig.h>
#include <common/events/Events.h>
#include <drivers/interrupt/external_interrupts.h>
#include <events/EventBroker.h>

using namespace std;
using namespace miosix;
using namespace Boardcore;
using namespace Main::SensorsConfig;
using namespace Common;

// BMX160 Watermark interrupt
void __attribute__((used)) EXTI3_IRQHandlerImpl()
{
    if (Main::Sensors::getInstance().bmx160 != nullptr)
        Main::Sensors::getInstance().bmx160->IRQupdateTimestamp(
            TimestampTimer::getTimestamp());
}

namespace Main
{

bool Sensors::start()
{
    GpioPin bmx160IntPin = miosix::sensors::bmx160::intr::getPin();
    enableExternalInterrupt(bmx160IntPin.getPort(), bmx160IntPin.getNumber(),
                            InterruptTrigger::FALLING_EDGE);

    return sensorManager->start();
}

bool Sensors::isStarted() { return sensorManager->areAllSensorsInitialized(); }

BMX160Data Sensors::getBMX160LastSample()
{
    miosix::PauseKernelLock lock;
    return bmx160 != nullptr ? bmx160->getLastSample() : BMX160Data{};
}

BMX160WithCorrectionData Sensors::getBMX160WithCorrectionLastSample()
{
    miosix::PauseKernelLock lock;
    return bmx160WithCorrection != nullptr
               ? bmx160WithCorrection->getLastSample()
               : BMX160WithCorrectionData{};
}

MPU9250Data Sensors::getMPU9250LastSample()
{
    PauseKernelLock lock;
    return mpu9250 != nullptr ? mpu9250->getLastSample() : MPU9250Data{};
}

MS5803Data Sensors::getMS5803LastSample()
{
    PauseKernelLock lock;
    return ms5803 != nullptr ? ms5803->getLastSample() : MS5803Data{};
}

ADS131M04Data Sensors::getADS131M04LastSample()
{
    PauseKernelLock lock;
    return ads131m04 != nullptr ? ads131m04->getLastSample() : ADS131M04Data{};
}

MPXH6115AData Sensors::getStaticPressureLastSample()
{
    PauseKernelLock lock;
    return staticPressure != nullptr ? staticPressure->getLastSample()
                                     : MPXH6115AData{};
}

MPXH6400AData Sensors::getDplPressureLastSample()
{
    PauseKernelLock lock;
    return dplPressure != nullptr ? dplPressure->getLastSample()
                                  : MPXH6400AData{};
}

AnalogLoadCellData Sensors::getLoadCellLastSample()
{
    PauseKernelLock lock;
    return loadCell != nullptr ? loadCell->getLastSample()
                               : AnalogLoadCellData{};
}

BatteryVoltageSensorData Sensors::getBatteryVoltageLastSample()
{
    PauseKernelLock lock;
    return batteryVoltage != nullptr ? batteryVoltage->getLastSample()
                                     : BatteryVoltageSensorData{};
}

InternalADCData Sensors::getInternalADCLastSample()
{
    PauseKernelLock lock;
    return internalAdc != nullptr ? internalAdc->getLastSample()
                                  : InternalADCData{};
}

void Sensors::calibrate()
{
    calibrating = true;

    ms5803Stats.reset();
    staticPressureStats.reset();
    dplPressureStats.reset();
    loadCellStats.reset();

    bmx160WithCorrection->startCalibration();

    Thread::sleep(CALIBRATION_DURATION);

    bmx160WithCorrection->stopCalibration();

    // Calibrate the analog pressure sensor to the digital one
    float ms5803Mean         = ms5803Stats.getStats().mean;
    float staticPressureMean = staticPressureStats.getStats().mean;
    float dplPressureMean    = dplPressureStats.getStats().mean;
    float loadCellMean       = loadCellStats.getStats().mean;
    staticPressure->setOffset(staticPressureMean - ms5803Mean);
    dplPressure->setOffset(dplPressureMean - ms5803Mean);
    loadCell->setOffset(loadCellMean);

    calibrating = false;
}

map<string, bool> Sensors::getSensorsState()
{
    map<string, bool> sensorsState;

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
    mpu9250Init();
    ms5803Init();
    ads131m04Init();
    staticPressureInit();
    dplPressureInit();
    loadCellInit();
    batteryVoltageInit();
    internalAdcInit();

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
    delete bmx160WithCorrection;
    delete mpu9250;
    delete ms5803;
    delete ads131m04;
    delete staticPressure;
    delete dplPressure;
    delete internalAdc;
    delete batteryVoltage;

    sensorManager->stop();
    delete sensorManager;
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
        new BMX160(Buses::getInstance().spi4,
                   miosix::sensors::bmx160::cs::getPin(), config, spiConfig);

    SensorInfo info("BMX160", SAMPLE_PERIOD_IMU_BMX,
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
        "BMX160WithCorrection", SAMPLE_PERIOD_IMU_BMX,
        [&]()
        { Logger::getInstance().log(bmx160WithCorrection->getLastSample()); });

    sensorsMap.emplace(make_pair(bmx160WithCorrection, info));

    LOG_INFO(logger, "BMX160WithCorrection setup done!");
}

void Sensors::mpu9250Init()
{
    mpu9250 =
        new MPU9250(Buses::getInstance().spi4, sensors::mpu9250::cs::getPin());

    SensorInfo info("MPU9250", SAMPLE_PERIOD_IMU_MPU,
                    bind(&Sensors::bmx160Callback, this));

    sensorsMap.emplace(make_pair(mpu9250, info));

    LOG_INFO(logger, "MPU9250 setup done!");
}

void Sensors::ms5803Init()
{
    SPIBusConfig spiConfig{};
    spiConfig.mode         = SPI::Mode::MODE_3;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    ms5803 = new MS5803(Buses::getInstance().spi2,
                        miosix::sensors::ms5803::cs::getPin(), spiConfig,
                        TEMP_DIVIDER_PRESS_DIGITAL);

    SensorInfo info(
        "MS5803", SAMPLE_PERIOD_PRESS_DIGITAL,
        [&]()
        {
            Logger::getInstance().log(ms5803->getLastSample());

            if (calibrating && ms5803->getLastSample().pressure != 0)
                ms5803Stats.add(ms5803->getLastSample().pressure);
        });

    sensorsMap.emplace(make_pair(ms5803, info));

    LOG_INFO(logger, "MS5803 setup done!");
}

void Sensors::ads131m04Init()
{
    SPIBusConfig spiConfig = ADS131M04::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;

    ads131m04 =
        new ADS131M04(Buses::getInstance().spi1,
                      miosix::sensors::ads131m04::cs1::getPin(), spiConfig);

    ads131m04->enableChannel(ADS131M04::Channel::CHANNEL_0);
    ads131m04->enableChannel(ADS131M04::Channel::CHANNEL_1);
    ads131m04->enableChannel(ADS131M04::Channel::CHANNEL_2);
    ads131m04->enableChannel(ADS131M04::Channel::CHANNEL_3);
    ads131m04->enableGlobalChopMode();
    ads131m04->setOversamplingRatio(ADS131M04::OversamplingRatio::OSR_4096);

    SensorInfo info("ADS131M04", SAMPLE_PERIOD_ADC_ADS131M04,
                    [&]()
                    { Logger::getInstance().log(ads131m04->getLastSample()); });

    sensorsMap.emplace(make_pair(ads131m04, info));

    LOG_INFO(logger, "ADS131M04 setup done!");
}

void Sensors::staticPressureInit()
{
    function<ADCData()> getVoltage(
        bind(&ADS131M04::getVoltage, ads131m04, ADC_CH_STATIC_PORT));

    staticPressure = new MPXH6115A(getVoltage, REFERENCE_VOLTAGE);

    SensorInfo info(
        "STATIC_PRESSURE", SAMPLE_PERIOD_ADC_ADS131M04,
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
    function<ADCData()> getVoltage(
        bind(&ADS131M04::getVoltage, ads131m04, ADC_CH_DPL_PORT));

    dplPressure = new MPXH6400A(getVoltage, REFERENCE_VOLTAGE);

    SensorInfo info(
        "DPL_PRESSURE", SAMPLE_PERIOD_ADC_ADS131M04,
        [&]()
        {
            Logger::getInstance().log(dplPressure->getLastSample());

            if (calibrating)
                dplPressureStats.add(dplPressure->getLastSample().pressure);
        });

    sensorsMap.emplace(make_pair(dplPressure, info));

    LOG_INFO(logger, "Deployment pressure sensor setup done!");
}

void Sensors::loadCellInit()
{
    function<ADCData()> getVoltage(
        bind(&ADS131M04::getVoltage, ads131m04, ADC_CH_LOAD_CELL));

    loadCell =
        new AnalogLoadCell(getVoltage, LOAD_CELL_MV_TO_V, LOAD_CELL_FULL_SCALE,
                           LOAD_CELL_SUPPLY_VOLTAGE);

    SensorInfo info("LOAD_CELL", SAMPLE_PERIOD_ADC_ADS131M04,
                    [&]()
                    {
                        Logger::getInstance().log(loadCell->getLastSample());

                        if (calibrating)
                            loadCellStats.add(loadCell->getLastSample().load);
                    });

    sensorsMap.emplace(make_pair(loadCell, info));

    LOG_INFO(logger, "Load cell sensor setup done!");
}

void Sensors::batteryVoltageInit()
{
    function<ADCData()> getVoltage(
        bind(&ADS131M04::getVoltage, ads131m04, ADC_CH_VBAT));
    batteryVoltage =
        new BatteryVoltageSensor(getVoltage, BATTERY_VOLTAGE_COEFF);

    SensorInfo info(
        "BATTERY_VOLTAGE", SAMPLE_PERIOD_ADC_ADS131M04,
        [&]() { Logger::getInstance().log(batteryVoltage->getLastSample()); });

    sensorsMap.emplace(make_pair(batteryVoltage, info));

    LOG_INFO(logger, "Battery voltage sensor setup done!");
}

void Sensors::internalAdcInit()
{
    internalAdc = new InternalADC(ADC3, INTERNAL_ADC_VREF);

    internalAdc->enableChannel(INTERNAL_ADC_CH_5V_CURRENT);
    internalAdc->enableChannel(INTERNAL_ADC_CH_CUTTER_CURRENT);

    SensorInfo info("INTERNAL_ADC", SAMPLE_PERIOD_INTERNAL_ADC);

    sensorsMap.emplace(make_pair(internalAdc, info));

    LOG_INFO(logger, "Internal ADC setup done!");
}

}  // namespace Main
