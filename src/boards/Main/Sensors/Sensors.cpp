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
#include <Main/FlightStatsRecorder/FlightStatsRecorder.h>
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
    GpioPin bmx160IntPin = sensors::bmx160::intr::getPin();
    enableExternalInterrupt(bmx160IntPin.getPort(), bmx160IntPin.getNumber(),
                            InterruptTrigger::FALLING_EDGE);

    return sensorManager->start();
}

bool Sensors::isStarted()
{
    return sensorManager->getSensorInfo(bmx160).isInitialized &&
           sensorManager->getSensorInfo(ms5803).isInitialized &&
           sensorManager->getSensorInfo(ubxGps).isInitialized;
    // return sensorManager->areAllSensorsInitialized();
}

void Sensors::setPitotData(Boardcore::PitotData data)
{
    PauseKernelLock lock;
    pitotData = data;

    FlightStatsRecorder::getInstance().update(data);
}

BMX160Data Sensors::getBMX160LastSample()
{
    PauseKernelLock lock;
    return bmx160 != nullptr ? bmx160->getLastSample() : BMX160Data{};
}

BMX160WithCorrectionData Sensors::getBMX160WithCorrectionLastSample()
{
    PauseKernelLock lock;

#ifndef HILSimulation
    return bmx160WithCorrection != nullptr
               ? bmx160WithCorrection->getLastSample()
               : BMX160WithCorrectionData{};
#else
    auto imuData = state.imu->getLastSample();
    BMX160WithCorrectionData data;

    data.accelerationTimestamp    = imuData.accelerationTimestamp;
    data.accelerationX            = imuData.accelerationX;
    data.accelerationY            = imuData.accelerationY;
    data.accelerationZ            = imuData.accelerationZ;
    data.angularVelocityTimestamp = imuData.angularVelocityTimestamp;
    data.angularVelocityX         = imuData.angularVelocityX;
    data.angularVelocityY         = imuData.angularVelocityY;
    data.angularVelocityZ         = imuData.angularVelocityZ;
    data.magneticFieldTimestamp   = imuData.magneticFieldTimestamp;
    data.magneticFieldX           = imuData.magneticFieldX;
    data.magneticFieldY           = imuData.magneticFieldY;
    data.magneticFieldZ           = imuData.magneticFieldZ;

    return data;
#endif
}

MPU9250Data Sensors::getMPU9250LastSample()
{
    PauseKernelLock lock;
    return mpu9250 != nullptr ? mpu9250->getLastSample() : MPU9250Data{};
}

MS5803Data Sensors::getMS5803LastSample()
{
    PauseKernelLock lock;

#ifndef HILSimulation
    return ms5803 != nullptr ? ms5803->getLastSample() : MS5803Data{};
#else
    auto baroData = state.barometer->getLastSample();
    auto tempData = state.temperature->getLastSample();

    return MS5803Data(baroData.pressureTimestamp, baroData.pressure,
                      tempData.temperatureTimestamp, tempData.temperature);
#endif
}

UBXGPSData Sensors::getUbxGpsLastSample()
{
    PauseKernelLock lock;

#ifndef HILSimulation
    return ubxGps != nullptr ? ubxGps->getLastSample() : UBXGPSData{};
#else
    auto data = state.gps->getLastSample();
    UBXGPSData ubxData;

    ubxData.gpsTimestamp  = data.gpsTimestamp;
    ubxData.latitude      = data.latitude;
    ubxData.longitude     = data.longitude;
    ubxData.height        = data.height;
    ubxData.velocityNorth = data.velocityNorth;
    ubxData.velocityEast  = data.velocityEast;
    ubxData.velocityDown  = data.velocityDown;
    ubxData.speed         = data.speed;
    ubxData.track         = data.track;
    ubxData.positionDOP   = data.positionDOP;
    ubxData.satellites    = data.satellites;
    ubxData.fix           = data.fix;

    return ubxData;
#endif
}

VN100Data Sensors::getVN100LastSample()
{
    PauseKernelLock lock;
    return vn100 != nullptr ? vn100->getLastSample() : VN100Data{};
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

Boardcore::PitotData Sensors::getPitotLastSample()
{
    PauseKernelLock lock;

#ifndef HILSimulation
    return pitotData;
#else
    return state.pitot->getLastSample();
#endif
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

bool Sensors::isCutterPresent()
{
    return cutterSensingMean > CUTTER_SENSING_THRESHOLD;
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
    staticPressure->updateOffset(staticPressureMean - ms5803Mean);
    dplPressure->updateOffset(dplPressureMean - ms5803Mean);
    loadCell->updateOffset(loadCellMean);

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
    mpu9250Init();
    ms5803Init();
    ubxGpsInit();
    ads131m04Init();
    staticPressureInit();
    dplPressureInit();
    loadCellInit();
    batteryVoltageInit();
    internalAdcInit();

    // Moved down here because the bmx takes some times to start
    bmx160Init();
    bmx160WithCorrectionInit();

#ifdef HILSimulation
    // Definition of the fake sensors for the simulation
    state.accelerometer = new HILAccelerometer(N_DATA_ACCEL);
    state.barometer     = new HILBarometer(N_DATA_BARO);
    state.pitot         = new HILPitot(N_DATA_PITOT);
    state.gps           = new HILGps(N_DATA_GPS);
    state.gyro          = new HILGyroscope(N_DATA_GYRO);
    state.magnetometer  = new HILMagnetometer(N_DATA_MAGN);
    state.imu           = new HILImu(N_DATA_IMU);
    state.temperature   = new HILTemperature(N_DATA_TEMP);
    state.kalman        = new HILKalman(N_DATA_KALM);

    sensorsMap = {{state.accelerometer, accelConfig},
                  {state.barometer, baroConfig},
                  {state.pitot, pitotConfig},
                  {state.magnetometer, magnConfig},
                  {state.imu, imuConfig},
                  {state.gps, gpsConfig},
                  {state.gyro, gyroConfig},
                  {state.temperature, tempConfig},
                  {state.kalman, kalmConfig}};
#endif

    // Create the sensor manager
    sensorManager = new SensorManager(sensorsMap);
}

Sensors::~Sensors()
{
    delete bmx160;
    delete bmx160WithCorrection;
    delete mpu9250;
    delete ms5803;
    delete ubxGps;
    delete ads131m04;
    delete staticPressure;
    delete dplPressure;
    delete internalAdc;
    delete batteryVoltage;

#ifdef HILSimulation
    delete state.accelerometer;
    delete state.barometer;
    delete state.pitot;
    delete state.gps;
    delete state.gyro;
    delete state.magnetometer;
    delete state.imu;
    delete state.temperature;
    delete state.kalman;
#endif

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

    config.accelerometerRange = IMU_BMX_ACC_FSR_ENUM;
    config.gyroscopeRange     = IMU_BMX_GYRO_FSR_ENUM;

    config.accelerometerDataRate = IMU_BMX_ACC_GYRO_ODR_ENUM;
    config.gyroscopeDataRate     = IMU_BMX_ACC_GYRO_ODR_ENUM;
    config.magnetometerRate      = IMU_BMX_MAG_ODR_ENUM;

    config.gyroscopeUnit = BMX160Config::GyroscopeMeasureUnit::RAD;

    bmx160 = new BMX160(Buses::getInstance().spi4,
                        sensors::bmx160::cs::getPin(), config, spiConfig);

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
        {
            Logger::getInstance().log(bmx160WithCorrection->getLastSample());
            FlightStatsRecorder::getInstance().update(
                bmx160WithCorrection->getLastSample());
        });

    sensorsMap.emplace(make_pair(bmx160WithCorrection, info));

    LOG_INFO(logger, "BMX160WithCorrection setup done!");
}

void Sensors::mpu9250Init()
{
    auto spiConfig         = MPU9250::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_8;

    mpu9250 = new MPU9250(
        Buses::getInstance().spi4, sensors::mpu9250::cs::getPin(), spiConfig,
        1000 / SAMPLE_PERIOD_VN100, IMU_MPU_GYRO_FSR, IMU_MPU_ACC_FSR);

    SensorInfo info("MPU9250", SAMPLE_PERIOD_IMU_MPU,
                    [&]()
                    { Logger::getInstance().log(mpu9250->getLastSample()); });

    sensorsMap.emplace(make_pair(mpu9250, info));

    LOG_INFO(logger, "MPU9250 setup done!");
}

void Sensors::ms5803Init()
{
    SPIBusConfig spiConfig{};
    spiConfig.mode         = SPI::Mode::MODE_3;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    ms5803 =
        new MS5803(Buses::getInstance().spi2, sensors::ms5803::cs::getPin(),
                   spiConfig, TEMP_DIVIDER_PRESS_DIGITAL);

    SensorInfo info(
        "MS5803", SAMPLE_PERIOD_PRESS_DIGITAL,
        [&]()
        {
            Logger::getInstance().log(ms5803->getLastSample());
            FlightStatsRecorder::getInstance().update(ms5803->getLastSample());

            if (calibrating && ms5803->getLastSample().pressure != 0)
                ms5803Stats.add(ms5803->getLastSample().pressure);
        });

    sensorsMap.emplace(make_pair(ms5803, info));

    LOG_INFO(logger, "MS5803 setup done!");
}

void Sensors::ubxGpsInit()
{
    ubxGps =
        new UBXGPSSpi(Buses::getInstance().spi2, sensors::gps::cs::getPin(),
                      UBXGPSSpi::getDefaultSPIConfig(), GPS_SAMPLE_RATE);

    SensorInfo info("UBXGPS", SAMPLE_PERIOD_GPS,
                    [&]()
                    { Logger::getInstance().log(ubxGps->getLastSample()); });
    sensorsMap.emplace(make_pair(ubxGps, info));

    LOG_INFO(logger, "UbloxGPS setup done!");
}

void Sensors::vn100Init()
{
    vn100 = new VN100(USART2, USARTInterface::Baudrate::B921600,
                      VN100::CRCOptions::CRC_ENABLE_16);

    SensorInfo info("VN100", SAMPLE_PERIOD_VN100,
                    [&]()
                    { Logger::getInstance().log(vn100->getLastSample()); });
    sensorsMap.emplace(make_pair(ubxGps, info));
}

void Sensors::ads131m04Init()
{
    SPIBusConfig spiConfig = ADS131M04::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;

    ads131m04 = new ADS131M04(Buses::getInstance().spi1,
                              sensors::ads131m04::cs1::getPin(), spiConfig);

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
            FlightStatsRecorder::getInstance().updateDplVane(
                dplPressure->getLastSample());

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
    internalAdc = new InternalADC(ADC1, INTERNAL_ADC_VREF);

    internalAdc->enableChannel(INTERNAL_ADC_CH_5V_CURRENT);
    internalAdc->enableChannel(INTERNAL_ADC_CH_CUTTER_CURRENT);
    internalAdc->enableChannel(INTERNAL_ADC_CH_CUTTER_SENSE);

    SensorInfo info(
        "INTERNAL_ADC", SAMPLE_PERIOD_INTERNAL_ADC,
        [&]()
        {
            Logger::getInstance().log(
                internalAdc->getVoltage(INTERNAL_ADC_CH_5V_CURRENT));
            Logger::getInstance().log(
                internalAdc->getVoltage(INTERNAL_ADC_CH_CUTTER_CURRENT));

            auto cutterSenseData =
                internalAdc->getVoltage(INTERNAL_ADC_CH_CUTTER_SENSE);
            Logger::getInstance().log(cutterSenseData);

            cutterSensingMean =
                cutterSensingMean * (1 - CUTTER_SENSING_MOV_MEAN_COEFF);
            cutterSensingMean +=
                cutterSenseData.voltage * CUTTER_SENSING_MOV_MEAN_COEFF;
        });

    sensorsMap.emplace(make_pair(internalAdc, info));

    LOG_INFO(logger, "Internal ADC setup done!");
}

}  // namespace Main
