/**
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta (luca.erbetta@skywarder.eu)
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Sensors.h"

#include <Debug.h>
#include <drivers/interrupt/external_interrupts.h>
#include <interfaces-impl/hwmapping.h>

#include <functional>
#include <utility>

#include "ADA/ADAController.h"
#include "DeathStack.h"
#include "LoggerService/LoggerService.h"
#include "TimestampTimer.h"
#include "configs/SensorManagerConfig.h"
#include "utils/aero/AeroUtils.h"

using std::bind;
using std::function;

// BMX160 Watermark interrupt
void __attribute__((used)) EXTI5_IRQHandlerImpl()
{
    using namespace DeathStackBoard;

    if (DeathStack::getInstance()->sensors->imu_bmx160 != nullptr)
    {
        DeathStack::getInstance()->sensors->imu_bmx160->IRQupdateTimestamp(
            TimestampTimer::getTimestamp());
    }
}

namespace DeathStackBoard
{
using namespace SensorConfigs;

Sensors::Sensors(SPIBusInterface& spi1_bus, TaskScheduler* scheduler)
    : spi1_bus(spi1_bus)
{
    // sensors are added to the map ordered by increasing period
    ADS1118Init();  // 6 ms
#ifdef HARDWARE_IN_THE_LOOP
    hilBarometerInit();
#else
    pressDigiInit();  // 10 ms
#endif
    magLISinit();
#ifdef HARDWARE_IN_THE_LOOP
    hilImuInit();
#else
    imuBMXInit();     // 23 ms
    imuBMXWithCorrectionInit();
#endif
    pressPitotInit();  // 24 ms
    pressDPLVaneInit();
    pressStaticInit();
#ifdef HARDWARE_IN_THE_LOOP
    hilGpsInit();
#else
    gpsUbloxInit();  // 40 ms
#endif
    internalAdcInit();  // 50 ms
    batteryVoltageInit();
    primaryCutterCurrentInit();
    backupCutterCurrentInit();

    sensor_manager = new SensorManager(scheduler, sensors_map);
}

Sensors::~Sensors()
{
#ifdef HARDWARE_IN_THE_LOOP
    delete hil_imu;
    delete hil_baro;
    delete hil_gps;
#else
    delete imu_bmx160;
    delete press_digital;
    delete gps_ublox;
#endif
    delete internal_adc;
    delete cs_cutter_primary;
    delete cs_cutter_backup;
    delete battery_voltage;
    delete adc_ads1118;
    delete press_pitot;
    delete press_dpl_vane;
    delete press_static_port;
    delete mag_lis3mdl;

    sensor_manager->stop();
    delete sensor_manager;
}

bool Sensors::start()
{
#ifndef HARDWARE_IN_THE_LOOP
    GpioPin int_pin = miosix::sensors::bmx160::intr::getPin();

    enableExternalInterrupt(int_pin.getPort(), int_pin.getNumber(),
                            InterruptTrigger::FALLING_EDGE);
    gps_ublox->start();
#endif

    bool sm_start_result = sensor_manager->start();

    // if not init ok, set failing sensors in sensors status
    if (!sm_start_result)
    {
        updateSensorsStatus();
    }

    LoggerService::getInstance()->log(status);

    return sm_start_result;
}

void calibrate() {}

void Sensors::internalAdcInit()
{
    internal_adc = new InternalADC(*ADC3, INTERNAL_ADC_VREF);

    internal_adc->enableChannel(ADC_CS_CUTTER_PRIMARY);
    internal_adc->enableChannel(ADC_CS_CUTTER_BACKUP);
    internal_adc->enableChannel(ADC_BATTERY_VOLTAGE);

    SensorInfo info("InternalADC", SAMPLE_PERIOD_INTERNAL_ADC,
                    bind(&Sensors::internalAdcCallback, this), false, true);
    sensors_map.emplace(std::make_pair(internal_adc, info));

    LOG_INFO(log, "InternalADC setup done!");
}

void Sensors::batteryVoltageInit()
{
    function<ADCData()> voltage_fun(
        bind(&InternalADC::getVoltage, internal_adc, ADC_BATTERY_VOLTAGE));
    battery_voltage =
        new BatteryVoltageSensor(voltage_fun, BATTERY_VOLTAGE_COEFF);

    SensorInfo info("BatterySensor", SAMPLE_PERIOD_INTERNAL_ADC,
                    bind(&Sensors::batteryVoltageCallback, this), false, true);

    sensors_map.emplace(std::make_pair(battery_voltage, info));

    LOG_INFO(log, "Battery voltage sensor setup done!");
}

void Sensors::primaryCutterCurrentInit()
{
    function<ADCData()> voltage_fun(
        bind(&InternalADC::getVoltage, internal_adc, ADC_CS_CUTTER_PRIMARY));
    function<float(float)> adc_to_current = [](float adc_in)
    {
        float current =
            CS_CURR_DKILIS * (adc_in / CS_CURR_RIS - CS_CURR_IISOFF);
        if (current < 0)
        {
            return (float)0;
        }
        return current;
    };
    cs_cutter_primary = new CurrentSensor(voltage_fun, adc_to_current);

    SensorInfo info("PrimaryCutterSensor", SAMPLE_PERIOD_INTERNAL_ADC,
                    bind(&Sensors::primaryCutterCurrentCallback, this), false,
                    true);

    sensors_map.emplace(std::make_pair(cs_cutter_primary, info));

    LOG_INFO(log, "Primary cutter current sensor setup done!");
}

void Sensors::backupCutterCurrentInit()
{
    function<ADCData()> voltage_fun(
        bind(&InternalADC::getVoltage, internal_adc, ADC_CS_CUTTER_BACKUP));
    function<float(float)> adc_to_current = [](float adc_in)
    {
        float current =
            CS_CURR_DKILIS * (adc_in / CS_CURR_RIS - CS_CURR_IISOFF);
        if (current < 0)
        {
            return (float)0;
        }
        return current;
    };
    cs_cutter_backup = new CurrentSensor(voltage_fun, adc_to_current);

    SensorInfo info("BackupCutterSensor", SAMPLE_PERIOD_INTERNAL_ADC,
                    bind(&Sensors::backupCutterCurrentCallback, this), false,
                    true);

    sensors_map.emplace(std::make_pair(cs_cutter_backup, info));

    LOG_INFO(log, "Backup cutter current sensor setup done!");
}

void Sensors::pressDigiInit()
{
    SPIBusConfig spi_cfg{};
    spi_cfg.clock_div = SPIClockDivider::DIV16;

    press_digital =
        new MS580301BA07(spi1_bus, miosix::sensors::ms5803::cs::getPin(),
                         spi_cfg, TEMP_DIVIDER_PRESS_DIGITAL);

    SensorInfo info("DigitalBarometer", SAMPLE_PERIOD_PRESS_DIGITAL,
                    bind(&Sensors::pressDigiCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_digital, info));

    LOG_INFO(log, "MS5803 pressure sensor setup done!");
}

void Sensors::ADS1118Init()
{
    SPIBusConfig spi_cfg = ADS1118::getDefaultSPIConfig();
    spi_cfg.clock_div    = SPIClockDivider::DIV64;

    ADS1118::ADS1118Config ads1118Config = ADS1118::ADS1118_DEFAULT_CONFIG;
    ads1118Config.bits.mode = ADS1118::ADS1118Mode::CONTIN_CONV_MODE;

    adc_ads1118 = new ADS1118(spi1_bus, miosix::sensors::ads1118::cs::getPin(),
                              ads1118Config, spi_cfg);

    adc_ads1118->enableInput(ADC_CH_STATIC_PORT, ADC_DR_STATIC_PORT,
                             ADC_PGA_STATIC_PORT);

    adc_ads1118->enableInput(ADC_CH_PITOT_PORT, ADC_DR_PITOT_PORT,
                             ADC_PGA_PITOT_PORT);
    adc_ads1118->enableInput(ADC_CH_DPL_PORT, ADC_DR_DPL_PORT,
                             ADC_PGA_DPL_PORT);

    adc_ads1118->enableInput(ADC_CH_VREF, ADC_DR_VREF, ADC_PGA_VREF);

    SensorInfo info("ADS1118", SAMPLE_PERIOD_ADC_ADS1118,
                    bind(&Sensors::ADS1118Callback, this), false, true);
    sensors_map.emplace(std::make_pair(adc_ads1118, info));

    LOG_INFO(log, "ADS1118 setup done!");
}

void Sensors::pressPitotInit()
{
    function<ADCData()> voltage_fun(
        bind(&ADS1118::getVoltage, adc_ads1118, ADC_CH_PITOT_PORT));
    press_pitot = new SSCDRRN015PDA(voltage_fun, REFERENCE_VOLTAGE);

    SensorInfo info("PitotBarometer", SAMPLE_PERIOD_PRESS_PITOT,
                    bind(&Sensors::pressPitotCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_pitot, info));

    LOG_INFO(log, "Pitot pressure sensor setup done!");
}

void Sensors::pressDPLVaneInit()
{
    function<ADCData()> voltage_fun(
        bind(&ADS1118::getVoltage, adc_ads1118, ADC_CH_DPL_PORT));
    press_dpl_vane = new SSCDANN030PAA(voltage_fun, REFERENCE_VOLTAGE);

    SensorInfo info("DeploymentBarometer", SAMPLE_PERIOD_PRESS_DPL,
                    bind(&Sensors::pressDPLVaneCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_dpl_vane, info));

    LOG_INFO(log, "DPL pressure sensor setup done!");
}

void Sensors::pressStaticInit()
{
    function<ADCData()> voltage_fun(
        bind(&ADS1118::getVoltage, adc_ads1118, ADC_CH_STATIC_PORT));
    press_static_port = new MPXHZ6130A(voltage_fun, REFERENCE_VOLTAGE);

    SensorInfo info("StaticPortsBarometer", SAMPLE_PERIOD_PRESS_STATIC,
                    bind(&Sensors::pressStaticCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_static_port, info));

    LOG_INFO(log, "Static pressure sensor setup done!");
}

void Sensors::imuBMXInit()
{
    SPIBusConfig spi_cfg;
    spi_cfg.clock_div = SPIClockDivider::DIV8;

    BMX160Config bmx_config;
    bmx_config.fifo_mode      = BMX160Config::FifoMode::HEADER;
    bmx_config.fifo_watermark = IMU_BMX_FIFO_WATERMARK;
    bmx_config.fifo_int       = BMX160Config::FifoInterruptMode::PIN_INT1;

    bmx_config.temp_divider = 1;

    bmx_config.acc_range = IMU_BMX_ACC_FULLSCALE_ENUM;
    bmx_config.gyr_range = IMU_BMX_GYRO_FULLSCALE_ENUM;

    bmx_config.acc_odr = IMU_BMX_ACC_GYRO_ODR_ENUM;
    bmx_config.gyr_odr = IMU_BMX_ACC_GYRO_ODR_ENUM;
    bmx_config.mag_odr = IMU_BMX_MAG_ODR_ENUM;

    bmx_config.gyr_unit = BMX160Config::GyroscopeMeasureUnit::RAD;

    imu_bmx160 = new BMX160(spi1_bus, miosix::sensors::bmx160::cs::getPin(),
                            bmx_config, spi_cfg);

    SensorInfo info("BMX160", SAMPLE_PERIOD_IMU_BMX,
                    bind(&Sensors::imuBMXCallback, this), false, true);

    sensors_map.emplace(std::make_pair(imu_bmx160, info));

    LOG_INFO(log, "BMX160 Setup done!");
}

void Sensors::imuBMXWithCorrectionInit()
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

    imu_bmx160_with_correction = new BMX160WithCorrection(
        imu_bmx160, correctionParameters, BMX160_AXIS_ROTATION);

    SensorInfo info("BMX160WithCorrection", SAMPLE_PERIOD_IMU_BMX,
                    bind(&Sensors::imuBMXWithCorrectionCallback, this), false,
                    true);

    sensors_map.emplace(std::make_pair(imu_bmx160_with_correction, info));

    LOG_INFO(log, "BMX160WithCorrection Setup done!");
}

void Sensors::magLISinit()
{
    SPIBusConfig busConfig;
    busConfig.clock_div = SPIClockDivider::DIV32;

    LIS3MDL::Config config;
    config.odr                = MAG_LIS_ODR_ENUM;
    config.scale              = MAG_LIS_FULLSCALE;
    config.temperatureDivider = 1;

    mag_lis3mdl = new LIS3MDL(spi1_bus, miosix::sensors::lis3mdl::cs::getPin(),
                              busConfig, config);

    SensorInfo info("LIS3MDL", SAMPLE_PERIOD_MAG_LIS,
                    bind(&Sensors::magLISCallback, this), false, true);

    sensors_map.emplace(std::make_pair(mag_lis3mdl, info));

    LOG_INFO(log, "LIS3MDL Setup done!");
}

void Sensors::gpsUbloxInit()
{
    gps_ublox = new UbloxGPS(GPS_BAUD_RATE, GPS_SAMPLE_RATE);

    SensorInfo info("UbloxGPS", GPS_SAMPLE_PERIOD,
                    bind(&Sensors::gpsUbloxCallback, this), false, true);

    sensors_map.emplace(std::make_pair(gps_ublox, info));

    LOG_INFO(log, "Ublox GPS Setup done!");
}

void Sensors::internalAdcCallback()
{
    // LoggerService::getInstance()->log(internal_adc->getLastSample());
}

void Sensors::batteryVoltageCallback()
{
    /*float v = battery_voltage->getLastSample().bat_voltage;
    if (v < 10.5)
    {
        LOG_CRIT(log, "******* LOW BATTERY ******* \n Voltage = {:02f} \n", v);
    }*/

    LoggerService::getInstance()->log(battery_voltage->getLastSample());
}

void Sensors::primaryCutterCurrentCallback()
{
    LoggerService::getInstance()->log(cs_cutter_primary->getLastSample());
}

void Sensors::backupCutterCurrentCallback()
{
    LoggerService::getInstance()->log(cs_cutter_backup->getLastSample());
}

#ifdef HARDWARE_IN_THE_LOOP
void Sensors::hilBarometerInit()
{
    HILTransceiver* simulator = HIL::getInstance()->simulator;

    hil_baro = new HILBarometer(simulator, N_DATA_BARO);

    SensorInfo info_baro("HILBaro", HIL_BARO_PERIOD,
                         bind(&Sensors::hilBaroCallback, this), false, true);

    sensors_map.emplace(std::make_pair(hil_baro, info_baro));

    LOG_INFO(log, "HIL barometer setup done!");
}
void Sensors::hilImuInit()
{
    HILTransceiver* simulator = HIL::getInstance()->simulator;

    hil_imu = new HILImu(simulator, N_DATA_IMU);

    SensorInfo info_imu("HILImu", HIL_IMU_PERIOD,
                        bind(&Sensors::hilIMUCallback, this), false, true);

    sensors_map.emplace(std::make_pair(hil_imu, info_imu));

    LOG_INFO(log, "HIL IMU setup done!");
}

void Sensors::hilGpsInit()
{
    HILTransceiver* simulator = HIL::getInstance()->simulator;

    hil_gps = new HILGps(simulator, N_DATA_GPS);

    SensorInfo info_gps("HILGps", HIL_GPS_PERIOD,
                        bind(&Sensors::hilGPSCallback, this), false, true);

    sensors_map.emplace(std::make_pair(hil_gps, info_gps));

    LOG_INFO(log, "HIL GPS setup done!");
}
#endif

void Sensors::pressDigiCallback()
{
    LoggerService::getInstance()->log(press_digital->getLastSample());
}

void Sensors::ADS1118Callback()
{
    LoggerService::getInstance()->log(adc_ads1118->getLastSample());
}

void Sensors::pressPitotCallback()
{
    SSCDRRN015PDAData d = press_pitot->getLastSample();
    LoggerService::getInstance()->log(d);

    ADAReferenceValues rv =
        DeathStack::getInstance()
            ->state_machines->ada_controller->getReferenceValues();

    float v = sqrtf(2 * d.press /
                    aeroutils::relDensity(press_digital->getLastSample().press,
                                          rv.ref_pressure, rv.ref_altitude,
                                          rv.ref_temperature));

    AirSpeedPitot aspeed_data{TimestampTimer::getTimestamp(), v};
    LoggerService::getInstance()->log(aspeed_data);
}

void Sensors::pressDPLVaneCallback()
{
    LoggerService::getInstance()->log(press_dpl_vane->getLastSample());
}

void Sensors::pressStaticCallback()
{
    LoggerService::getInstance()->log(press_static_port->getLastSample());
}

void Sensors::imuBMXCallback()
{
    uint8_t fifo_size = imu_bmx160->getLastFifoSize();
    auto& fifo        = imu_bmx160->getLastFifo();

    LoggerService::getInstance()->log(imu_bmx160->getTemperature());

    for (uint8_t i = 0; i < fifo_size; ++i)
    {
        LoggerService::getInstance()->log(fifo.at(i));
    }

    // static unsigned int downsample_ctr = 0;

    // if (downsample_ctr++ % 20 == 0)
    // {
    //     auto sample = fifo.at(0);
    //     LOG_INFO(log.getChild("bmx160"),
    //                     "acc xyz: {:+.3f},{:+.3f},{:+.3f} gyro xyz: "
    //                     "{:+.3f},{:+.3f},{:+.3f}",
    //                     sample.accel_x, sample.accel_y, sample.accel_z,
    //                     sample.gyro_x, sample.gyro_y, sample.gyro_z);
    // }
}

void Sensors::imuBMXWithCorrectionCallback()
{

    LoggerService::getInstance()->log(
        imu_bmx160_with_correction->getLastSample());
}

void Sensors::magLISCallback()
{
    LoggerService::getInstance()->log(mag_lis3mdl->getLastSample());

    // static unsigned int downsample_ctr = 0;

    // if (downsample_ctr++ % 20 == 0)
    // {
    //     auto sample = mag_lis3mdl->getLastSample();
    //     LOG_INFO(log.getChild("lis3mdl"),
    //                     "mag xyzt: {:+.3f},{:+.3f},{:+.3f},{:+.3f}",
    //                     sample.mag_x, sample.mag_y, sample.mag_z,
    //                     sample.temp);
    // }
}

void Sensors::gpsUbloxCallback()
{
    LoggerService::getInstance()->log(gps_ublox->getLastSample());
}

#ifdef HARDWARE_IN_THE_LOOP
void Sensors::hilIMUCallback()
{
    LoggerService::getInstance()->log(hil_imu->getLastSample());
}
void Sensors::hilBaroCallback()
{
    LoggerService::getInstance()->log(hil_baro->getLastSample());
}
void Sensors::hilGPSCallback()
{
    LoggerService::getInstance()->log(hil_gps->getLastSample());
}
#endif

void Sensors::updateSensorsStatus()
{
    SensorInfo info;

    info = sensor_manager->getSensorInfo(imu_bmx160);
    if (!info.is_initialized)
    {
        status.bmx160 = 0;
    }

    info = sensor_manager->getSensorInfo(mag_lis3mdl);
    if (!info.is_initialized)
    {
        status.lis3mdl = 0;
    }

    info = sensor_manager->getSensorInfo(gps_ublox);
    if (!info.is_initialized)
    {
        status.gps = 0;
    }

    info = sensor_manager->getSensorInfo(internal_adc);
    if (!info.is_initialized)
    {
        status.internal_adc = 0;
    }

    info = sensor_manager->getSensorInfo(adc_ads1118);
    if (!info.is_initialized)
    {
        status.ads1118 = 0;
    }
}

}  // namespace DeathStackBoard