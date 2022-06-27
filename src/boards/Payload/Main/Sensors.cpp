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

#include <Payload/Configs/SensorsConfig.h>
#include <Payload/Payload.h>
#include <drivers/interrupt/external_interrupts.h>
#include <miosix.h>
#include <sensors/SensorInfo.h>

using namespace Boardcore;
using namespace std;

// BMX160 Watermark interrupt
void __attribute__((used)) EXTI5_IRQHandlerImpl()
{
    if (Payload::Payload::getInstance().sensors->imuBMX160 != nullptr)
    {
        Payload::Payload::getInstance().sensors->imuBMX160->IRQupdateTimestamp(
            TimestampTimer::getTimestamp());
    }
}

namespace Payload
{
Sensors::Sensors(SPIBusInterface& spiBus, TaskScheduler* scheduler)
    : spiBus(spiBus)
{
    // Register the SDlogger
    SDlogger = &Logger::getInstance();

    // Add the sensors to the map ordering them by increasing period
    adcADS1118Init();
    magnetometerLIS3MDLInit();
    imuBMX160Init();
    correctedImuBMX160Init();
    digitalPressureInit();
    pitotPressureInit();
    dplVanePressureInit();
    staticPortPressureInit();
    // TODO UNCOMMENT THIS
    //  gpsUbloxInit();
    internalAdcInit();
    batteryVoltageInit();

    // Now create the sensor manager with all the inserted sensors inside the
    // map
    sensorManager = new SensorManager(sensorsMap, scheduler);
}

Sensors::~Sensors()
{
    delete internalAdc;
    delete batteryVoltage;
    delete digitalPressure;
    delete adcADS1118;
    delete dplVanePressure;
    delete staticPortPressure;
    delete pitotPressure;
    delete imuBMX160;
    delete correctedImuBMX160;
    delete magnetometerLIS3MDL;
    delete gpsUblox;
}

bool Sensors::start()
{
    // BMX160 IMU enable the external interrupt on the correct pin
    miosix::GpioPin interruptPin = miosix::sensors::bmx160::intr::getPin();
    enableExternalInterrupt(interruptPin.getPort(), interruptPin.getNumber(),
                            InterruptTrigger::FALLING_EDGE, 0);

    // Start all the sensors, record the result and log it
    bool startResult = sensorManager->start();

    if (!startResult)
    {
        updateSensorsStatus();
    }

    // Log the sensors status
    SDlogger->log(status);
    return startResult;
}

void Sensors::calibrate()
{
    // Calibrate the IMU and log the biases result
    correctedImuBMX160->calibrate();
    SDlogger->log(correctedImuBMX160->getGyroscopeBiases());

    // TODO Decide calibration techinique
    // Mean some digital pressure samples to calibrate the analog sensors
    // float mean = 0;
    // for (unsigned int i = 0; i < STATIC_PRESS_CALIB_SAMPLES_NUM; i++)
    // {
    //     Thread::sleep(PRESS_DIGITAL_SAMPLE_PERIOD);
    //     mean += digitalPressure->getLastSample().pressure;
    // }
    // staticPortPressure->setReferencePressure(mean /
    //                                          STATIC_PRESS_CALIB_SAMPLES_NUM);
    // staticPortPressure->calibrate();

    // // Wait for differential and static barometers calibration
    // // TODO check the OR expression, it used to be an AND (?)
    // while (pitotPressure->isCalibrating() ||
    //        staticPortPressure->isCalibrating())
    // {
    //     Thread::sleep(10);
    // }
}

/**
 * INITS
 */

void Sensors::internalAdcInit()
{
    // Set the internal adc
    internalAdc = new InternalADC(ADC3, INTERNAL_ADC_VREF);
    internalAdc->enableChannel(ADC_BATTERY_VOLTAGE);

    // Create the sensor info
    SensorInfo info("InternalADC", INTERNAL_ADC_SAMPLE_PERIOD,
                    bind(&Sensors::internalAdcCallback, this));
    sensorsMap.emplace(make_pair(internalAdc, info));

    LOG_INFO(logger, "InternalADC setup done!");
}

void Sensors::batteryVoltageInit()
{
    // Crate a function that calls the internal ADC to read the battery voltage
    function<ADCData()> readVoltage(
        bind(&InternalADC::getVoltage, internalAdc, ADC_BATTERY_VOLTAGE));

    // Now i create a battery voltage sensor that uses the internal ADC
    batteryVoltage =
        new BatteryVoltageSensor(readVoltage, BATTERY_VOLTAGE_COEFF);

    // Create the sensor info
    SensorInfo info("BatterySensor", INTERNAL_ADC_SAMPLE_PERIOD,
                    bind(&Sensors::batteryVoltageCallback, this));
    sensorsMap.emplace(make_pair(batteryVoltage, info));

    LOG_INFO(logger, "Battery voltage sensor setup done!");
}

void Sensors::digitalPressureInit()
{
    // Setup the SPI
    SPIBusConfig spiConfig{};
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    // Set the digital barometer
    miosix::GpioPin cs = miosix::sensors::ms5803::cs::getPin();
    digitalPressure =
        new MS5803(spiBus, cs, spiConfig, PRESS_DIGITAL_TEMP_DIVIDER);

    // Create the sensor info
    SensorInfo info("DigitalBarometer", PRESS_DIGITAL_SAMPLE_PERIOD,
                    bind(&Sensors::digitalPressureCallback, this));
    sensorsMap.emplace(make_pair(digitalPressure, info));

    LOG_INFO(logger, "MS5803 pressure sensor setup done!");
}

void Sensors::adcADS1118Init()
{
    // Setup the SPI
    SPIBusConfig spiConfig = ADS1118::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;

    // Setup the ADC
    ADS1118::ADS1118Config adcConfig = ADS1118::ADS1118_DEFAULT_CONFIG;
    adcConfig.bits.mode = ADS1118::ADS1118Mode::CONTINUOUS_CONV_MODE;

    adcADS1118 = new ADS1118(spiBus, miosix::sensors::ads1118::cs::getPin(),
                             adcConfig, spiConfig);

    // Enable ADC ports
    adcADS1118->enableInput(ADC_CH_STATIC_PORT, ADC_DR_STATIC_PORT,
                            ADC_PGA_STATIC_PORT);
    adcADS1118->enableInput(ADC_CH_PITOT_PORT, ADC_DR_PITOT_PORT,
                            ADC_PGA_PITOT_PORT);
    adcADS1118->enableInput(ADC_CH_DPL_PORT, ADC_DR_DPL_PORT, ADC_PGA_DPL_PORT);
    adcADS1118->enableInput(ADC_CH_VREF, ADC_DR_VREF, ADC_PGA_VREF);

    // Create the sensor info
    SensorInfo info("ADS1118", ADC_ADS1118_SAMPLE_PERIOD,
                    bind(&Sensors::adcADS1118Callback, this));
    sensorsMap.emplace(make_pair(adcADS1118, info));

    LOG_INFO(logger, "ADS1118 adc setup done!");
}

void Sensors::pitotPressureInit()
{
    // Create a function to read the analog voltage
    function<ADCData()> readVoltage(
        bind(&ADS1118::getVoltage, adcADS1118, ADC_CH_PITOT_PORT));

    // Setup the pitot sensor
    pitotPressure = new SSCDRRN015PDA(readVoltage, REFERENCE_VOLTAGE);

    // Create the sensor info
    SensorInfo info("PitotBarometer", PITOT_PRESS_SAMPLE_PERIOD,
                    bind(&Sensors::pitotPressureCallback, this));

    sensorsMap.emplace(make_pair(pitotPressure, info));

    LOG_INFO(logger, "PITOT differential pressure sensor setup done!");
}

void Sensors::dplVanePressureInit()
{
    // Create a function to read the analog voltage
    function<ADCData()> readVoltage(
        bind(&ADS1118::getVoltage, adcADS1118, ADC_CH_DPL_PORT));

    // Setup the DPL sensor
    dplVanePressure = new SSCDANN030PAA(readVoltage, REFERENCE_VOLTAGE);

    // Create the sensor info
    SensorInfo info("DeploymentBarometer", DPL_PRESS_SAMPLE_PERIOD,
                    bind(&Sensors::dplVanePressureCallback, this));
    sensorsMap.emplace(make_pair(dplVanePressure, info));

    LOG_INFO(logger, "Deployment pressure sensor setup done!");
}

void Sensors::staticPortPressureInit()
{
    // Create a function to read analog voltage
    function<ADCData()> readVoltage(
        bind(&ADS1118::getVoltage, adcADS1118, ADC_CH_STATIC_PORT));

    // Setup the static port sensor
    staticPortPressure = new MPXHZ6130A(readVoltage, REFERENCE_VOLTAGE);

    // Create the sensor info
    SensorInfo info("StaticPortsBarometer", STATIC_PRESS_SAMPLE_PERIOD,
                    bind(&Sensors::staticPortPressureCallback, this));
    sensorsMap.emplace(make_pair(staticPortPressure, info));

    LOG_INFO(logger, "Static port pressure sensor setup done!");
}

void Sensors::imuBMX160Init()
{
    // Setup the SPI
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_8;

    // Setup the sensor
    BMX160Config bmxConfig;
    bmxConfig.fifoMode      = BMX160Config::FifoMode::HEADER;
    bmxConfig.fifoWatermark = IMU_BMX_FIFO_WATERMARK;
    bmxConfig.fifoInterrupt = BMX160Config::FifoInterruptPin::PIN_INT1;

    bmxConfig.temperatureDivider = IMU_BMX_TEMP_DIVIDER;

    bmxConfig.accelerometerRange = IMU_BMX_ACC_FULLSCALE_ENUM;
    bmxConfig.gyroscopeRange     = IMU_BMX_GYRO_FULLSCALE_ENUM;

    bmxConfig.accelerometerDataRate = IMU_BMX_ACC_GYRO_ODR_ENUM;
    bmxConfig.gyroscopeDataRate     = IMU_BMX_ACC_GYRO_ODR_ENUM;
    bmxConfig.magnetometerRate      = IMU_BMX_MAG_ODR_ENUM;

    bmxConfig.gyroscopeUnit = BMX160Config::GyroscopeMeasureUnit::RAD;

    imuBMX160 = new BMX160(spiBus, miosix::sensors::bmx160::cs::getPin(),
                           bmxConfig, spiConfig);

    // Create the sensor info
    SensorInfo info("BMX160", IMU_BMX_SAMPLE_PERIOD,
                    bind(&Sensors::imuBMX160Callback, this));
    sensorsMap.emplace(make_pair(imuBMX160, info));

    LOG_INFO(logger, "BMX160 Setup done!");
}

void Sensors::correctedImuBMX160Init()
{
    // Read the correction parameters
    BMX160CorrectionParameters correctionParameters =
        BMX160WithCorrection::readCorrectionParametersFromFile(
            BMX160_CORRECTION_PARAMETERS_FILE);

    // Print the calibration parameters
    TRACE("[Sensors] Current accelerometer bias vector\n");
    TRACE("[Sensors] b = [    % 2.5f    % 2.5f    % 2.5f    ]\n",
          correctionParameters.accelParams(0, 1),
          correctionParameters.accelParams(1, 1),
          correctionParameters.accelParams(2, 1));
    TRACE("[Sensors] Matrix to be multiplied to the input vector\n");
    TRACE("[Sensors]     |    % 2.5f    % 2.5f    % 2.5f    |\n",
          correctionParameters.accelParams(0, 0), 0.f, 0.f);
    TRACE("[Sensors] M = |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f,
          correctionParameters.accelParams(1, 0), 0.f);
    TRACE("[Sensors]     |    % 2.5f    % 2.5f    % 2.5f    |\n\n", 0.f, 0.f,
          correctionParameters.accelParams(2, 0));
    TRACE("[Sensors] Current magnetometer bias vector\n");
    TRACE("[Sensors] b = [    % 2.5f    % 2.5f    % 2.5f    ]\n",
          correctionParameters.magnetoParams(0, 1),
          correctionParameters.magnetoParams(1, 1),
          correctionParameters.magnetoParams(2, 1));
    TRACE("[Sensors] Matrix to be multiplied to the input vector\n");
    TRACE("[Sensors]     |    % 2.5f    % 2.5f    % 2.5f    |\n",
          correctionParameters.magnetoParams(0, 0), 0.f, 0.f);
    TRACE("[Sensors] M = |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f,
          correctionParameters.magnetoParams(1, 0), 0.f);
    TRACE("[Sensors]     |    % 2.5f    % 2.5f    % 2.5f    |\n\n", 0.f, 0.f,
          correctionParameters.magnetoParams(2, 0));
    TRACE(
        "[Sensors] The current minimun number of gyroscope samples for "
        "calibration is %d\n",
        correctionParameters.minGyroSamplesForCalibration);

    // Setup the sensor
    correctedImuBMX160 = new BMX160WithCorrection(
        imuBMX160, correctionParameters, BMX160_AXIS_ROTATION);

    // Create the sensor info
    SensorInfo info("BMX160WithCorrection", IMU_BMX_SAMPLE_PERIOD,
                    bind(&Sensors::correctedImuBMX160Callback, this));
    sensorsMap.emplace(make_pair(correctedImuBMX160, info));

    LOG_INFO(logger, "BMX160WithCorrection setup done!");
}

void Sensors::magnetometerLIS3MDLInit()
{
    // Configure the spi bus
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    // Configure the sensor
    LIS3MDL::Config sensorConfig;
    sensorConfig.odr                = MAG_LIS_ODR_ENUM;
    sensorConfig.scale              = MAG_LIS_FULLSCALE;
    sensorConfig.temperatureDivider = 1;

    // Setup the sensor
    magnetometerLIS3MDL =
        new LIS3MDL(spiBus, miosix::sensors::lis3mdl::cs::getPin(), spiConfig,
                    sensorConfig);

    // Create the sensor info
    SensorInfo info("LIS3MDL", MAG_LIS_SAMPLE_PERIOD,
                    bind(&Sensors::magnetometerLIS3MDLCallback, this));
    sensorsMap.emplace(make_pair(magnetometerLIS3MDL, info));

    LOG_INFO(logger, "LIS3MDL setup done!");
}

void Sensors::gpsUbloxInit()
{
    // On STACK V2 it is connected via serial interface
    gpsUblox = new UBXGPSSerial(GPS_BAUD_RATE, GPS_SAMPLE_RATE, 2, "gps", 9600);

    // Create the sensor info
    SensorInfo info("UbloxGPS", GPS_SAMPLE_PERIOD,
                    bind(&Sensors::gpsUbloxCallback, this));
    sensorsMap.emplace(make_pair(gpsUblox, info));

    LOG_INFO(logger, "UbloxGPS setup done!");
}

/**
 * CALLBACKS
 */
void Sensors::internalAdcCallback()
{
    // Log the thing
    SDlogger->log(internalAdc->getLastSample());
}

void Sensors::batteryVoltageCallback()
{
    // Log the thing
    SDlogger->log(batteryVoltage->getLastSample());
}

void Sensors::digitalPressureCallback()
{
    // Log the thing
    SDlogger->log(digitalPressure->getLastSample());
}

void Sensors::adcADS1118Callback()
{
    // Log the thing
    SDlogger->log(adcADS1118->getLastSample());
}

void Sensors::dplVanePressureCallback()
{
    // Log the thing
    SDlogger->log(dplVanePressure->getLastSample());
}

void Sensors::staticPortPressureCallback()
{
    // Log the thing
    SDlogger->log(staticPortPressure->getLastSample());
}

void Sensors::pitotPressureCallback()
{
    // Log the thing
    SDlogger->log(pitotPressure->getLastSample());
}

void Sensors::imuBMX160Callback()
{
    // Log the thing
    SDlogger->log(imuBMX160->getLastSample());
}

void Sensors::correctedImuBMX160Callback()
{
    // Log the thing
    SDlogger->log(correctedImuBMX160->getLastSample());
}

void Sensors::magnetometerLIS3MDLCallback()
{
    // Log the thing
    SDlogger->log(magnetometerLIS3MDL->getLastSample());
}

void Sensors::gpsUbloxCallback()
{
    // Log the thing
    SDlogger->log(gpsUblox->getLastSample());
}

void Sensors::updateSensorsStatus()
{
    // Check the various sensors
    SensorInfo info = sensorManager->getSensorInfo(imuBMX160);
    if (!info.isInitialized)
    {
        status.BMX160 = SensorDriverStatus::DRIVER_ERROR;
    }

    info = sensorManager->getSensorInfo(magnetometerLIS3MDL);
    if (!info.isInitialized)
    {
        status.LIS3MDL = SensorDriverStatus::DRIVER_ERROR;
    }

    info = sensorManager->getSensorInfo(gpsUblox);
    if (!info.isInitialized)
    {
        status.GPS = SensorDriverStatus::DRIVER_ERROR;
    }

    info = sensorManager->getSensorInfo(internalAdc);
    if (!info.isInitialized)
    {
        status.InternalADC = SensorDriverStatus::DRIVER_ERROR;
    }

    info = sensorManager->getSensorInfo(adcADS1118);
    if (!info.isInitialized)
    {
        status.ADS1118 = SensorDriverStatus::DRIVER_ERROR;
    }

    info = sensorManager->getSensorInfo(digitalPressure);
    if (!info.isInitialized)
    {
        status.MS5803 = SensorDriverStatus::DRIVER_ERROR;
    }
}

/**
 * KERNEL LOCK GETTERS
 */
Boardcore::InternalADCData Sensors::getInternalAdcLastSample()
{
    miosix::PauseKernelLock lock;
    return internalAdc->getLastSample();
}

Boardcore::BatteryVoltageSensorData Sensors::getBatteryVoltageLastSample()
{
    miosix::PauseKernelLock lock;
    return batteryVoltage->getLastSample();
}

Boardcore::MS5803Data Sensors::getDigitalPressureLastSample()
{
    miosix::PauseKernelLock lock;
    return digitalPressure->getLastSample();
}

Boardcore::ADS1118Data Sensors::getAdcADS1118LastSample()
{
    miosix::PauseKernelLock lock;
    return adcADS1118->getLastSample();
}

Boardcore::SSCDANN030PAAData Sensors::getDplVanePressureLastSample()
{
    miosix::PauseKernelLock lock;
    return dplVanePressure->getLastSample();
}

Boardcore::MPXHZ6130AData Sensors::getStaticPortPressureLastSample()
{
    miosix::PauseKernelLock lock;
    return staticPortPressure->getLastSample();
}

Boardcore::SSCDRRN015PDAData Sensors::getPitotPressureLastSample()
{
    miosix::PauseKernelLock lock;
    return pitotPressure->getLastSample();
}

Boardcore::BMX160Data Sensors::getImuBMX160LastSample()
{
    miosix::PauseKernelLock lock;
    return imuBMX160->getLastSample();
}

Boardcore::BMX160WithCorrectionData Sensors::getCorrectedImuBMX160LastSample()
{
    miosix::PauseKernelLock lock;
    return correctedImuBMX160->getLastSample();
}

Boardcore::LIS3MDLData Sensors::getMagnetometerLIS3MDLLastSample()
{
    miosix::PauseKernelLock lock;
    return magnetometerLIS3MDL->getLastSample();
}

Boardcore::UBXGPSData Sensors::getGPSLastSample()
{
    miosix::PauseKernelLock lock;
    return gpsUblox->getLastSample();
}

}  // namespace Payload