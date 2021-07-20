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

#include "DeathStack.h"
#include "LoggerService/LoggerService.h"
#include "TimestampTimer.h"
#include "configs/SensorManagerConfig.h"

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
#ifdef HARDWARE_IN_THE_LOOP
    hilSensorsInit();
#else
    // Pressure sensors
    internalAdcInit();
    batteryVoltageInit();
    primaryCutterCurrentInit();
    backupCutterCurrentInit();
    pressDigiInit();
    ADS1118Init();
    pressPitotInit();
    pressDPLVaneInit();
    pressStaticInit();
    imuBMXInit();
    magLISinit();
    gpsUbloxInit();
#endif

    sensor_manager = new SensorManager(scheduler, sensors_map);
}

Sensors::~Sensors()
{
#ifdef HARDWARE_IN_THE_LOOP
    delete hil_imu;
    delete hil_baro;
    delete hil_gps;
#else
    delete internal_adc;
    delete cs_cutter_primary;
    delete cs_cutter_backup;
    delete battery_voltage;
    delete press_digital;
    delete adc_ads1118;
    delete press_pitot;
    delete press_dpl_vane;
    delete press_static_port;
    delete imu_bmx160;
    delete mag_lis3mdl;
    delete gps_ublox;
#endif

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

    return sensor_manager->start();
}

void Sensors::internalAdcInit()
{
    internal_adc = new InternalADC(*ADC3, INTERNAL_ADC_VREF);

    internal_adc->enableChannel(ADC_CS_CUTTER_PRIMARY);
    internal_adc->enableChannel(ADC_CS_CUTTER_BACKUP);
    internal_adc->enableChannel(ADC_BATTERY_VOLTAGE);

    SensorInfo info("InternalADC", SAMPLE_PERIOD_INTERNAL_ADC,
                    bind(&Sensors::internalAdcCallback, this), false, true);
    sensors_map.emplace(std::make_pair(internal_adc, info));

    TRACE("InternalADC setup done! (%p)\n", internal_adc);
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

    TRACE("Battery voltage sensor setup done! (%p)\n", battery_voltage);
}

void Sensors::primaryCutterCurrentInit()
{
    function<ADCData()> voltage_fun(
        bind(&InternalADC::getVoltage, internal_adc, ADC_CS_CUTTER_PRIMARY));
    function<float(float)> adc_to_current = [](float adc_in) {
        return CS_CURR_DKILIS * (adc_in / CS_CURR_RIS - CS_CURR_IISOFF);
    };
    cs_cutter_primary = new CurrentSensor(voltage_fun, adc_to_current);

    SensorInfo info("PrimaryCutterSensor", SAMPLE_PERIOD_INTERNAL_ADC,
                    bind(&Sensors::primaryCutterCurrentCallback, this), false,
                    true);

    sensors_map.emplace(std::make_pair(cs_cutter_primary, info));

    TRACE("Primary cutter current sensor setup done! (%p)\n",
          cs_cutter_primary);
}

void Sensors::backupCutterCurrentInit()
{
    function<ADCData()> voltage_fun(
        bind(&InternalADC::getVoltage, internal_adc, ADC_CS_CUTTER_BACKUP));
    function<float(float)> adc_to_current = [](float adc_in) {
        return CS_CURR_DKILIS * (adc_in / CS_CURR_RIS - CS_CURR_IISOFF);
    };
    cs_cutter_backup = new CurrentSensor(voltage_fun, adc_to_current);

    SensorInfo info("BackupCutterSensor", SAMPLE_PERIOD_INTERNAL_ADC,
                    bind(&Sensors::backupCutterCurrentCallback, this), false,
                    true);

    sensors_map.emplace(std::make_pair(cs_cutter_backup, info));

    TRACE("Backup cutter current sensor setup done! (%p)\n", cs_cutter_backup);
}

void Sensors::pressDigiInit()
{
    SPIBusConfig spi_cfg{};
    spi_cfg.clock_div = SPIClockDivider::DIV16;

    press_digital = new MS580301BA07(
        spi1_bus, miosix::sensors::ms5803::cs::getPin(), spi_cfg);

    SensorInfo info("DigitalBarometer", SAMPLE_PERIOD_PRESS_DIGITAL,
                    bind(&Sensors::pressDigiCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_digital, info));

    TRACE("MS5803 pressure sensor setup done! (%p)\n", press_digital);
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

    TRACE("ADS1118 setup done! (%p)\n", adc_ads1118);
}

void Sensors::pressPitotInit()
{
    function<ADCData()> voltage_fun(
        bind(&ADS1118::getVoltage, adc_ads1118, ADC_CH_PITOT_PORT));
    press_pitot = new SSCDRRN015PDA(voltage_fun, REFERENCE_VOLTAGE);

    SensorInfo info("PitotBarometer", SAMPLE_PERIOD_PRESS_PITOT,
                    bind(&Sensors::pressPitotCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_pitot, info));

    TRACE("Pitot pressure sensor setup done! (%p)\n", press_pitot);
}

void Sensors::pressDPLVaneInit()
{
    function<ADCData()> voltage_fun(
        bind(&ADS1118::getVoltage, adc_ads1118, ADC_CH_DPL_PORT));
    press_dpl_vane = new SSCDANN030PAA(voltage_fun, REFERENCE_VOLTAGE);

    SensorInfo info("DeploymentBarometer", SAMPLE_PERIOD_PRESS_DPL,
                    bind(&Sensors::pressDPLVaneCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_dpl_vane, info));

    TRACE("DPL pressure sensor setup done! (%p)\n", press_dpl_vane);
}

void Sensors::pressStaticInit()
{
    function<ADCData()> voltage_fun(
        bind(&ADS1118::getVoltage, adc_ads1118, ADC_CH_STATIC_PORT));
    press_static_port = new MPXHZ6130A(voltage_fun, REFERENCE_VOLTAGE);

    SensorInfo info("StaticPortsBarometer", SAMPLE_PERIOD_PRESS_STATIC,
                    bind(&Sensors::pressStaticCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_static_port, info));

    TRACE("Static pressure sensor setup done! (%p)\n", press_static_port);
}

void Sensors::imuBMXInit()
{
    SPIBusConfig spi_cfg;
    spi_cfg.clock_div = SPIClockDivider::DIV8;

    BMX160Config bmx_config;
    bmx_config.fifo_mode      = BMX160Config::FifoMode::HEADER;
    bmx_config.fifo_watermark = IMU_BMX_FIFO_WATERMARK;
    bmx_config.fifo_int       = BMX160Config::FifoInt::PIN_INT1;

    bmx_config.temp_divider = 1;

    bmx_config.acc_range = IMU_BMX_ACC_FULLSCALE_ENUM;
    bmx_config.gyr_range = IMU_BMX_GYRO_FULLSCALE_ENUM;

    bmx_config.acc_odr = IMU_BMX_ACC_GYRO_ODR_ENUM;
    bmx_config.gyr_odr = IMU_BMX_ACC_GYRO_ODR_ENUM;
    bmx_config.mag_odr = IMU_BMX_MAG_ODR_ENUM;

    bmx_config.gyr_unit = BMX160Config::GyrMeasureUnit::RAD;

    imu_bmx160 = new BMX160(spi1_bus, miosix::sensors::bmx160::cs::getPin(),
                            bmx_config, spi_cfg);

    SensorInfo info("BMX160", SAMPLE_PERIOD_IMU_BMX,
                    bind(&Sensors::imuBMXCallback, this), false, true);

    sensors_map.emplace(std::make_pair(imu_bmx160, info));

    TRACE("BMX160 Setup done! (%p)\n", imu_bmx160);
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

    TRACE("LIS3MDL Setup done! (%p)\n", mag_lis3mdl);
}

void Sensors::gpsUbloxInit()
{
    gps_ublox = new UbloxGPS(GPS_BAUD_RATE);

    SensorInfo info("UbloxGPS", SAMPLE_PERIOD_GPS,
                    bind(&Sensors::gpsUbloxCallback, this), false, true);

    sensors_map.emplace(std::make_pair(gps_ublox, info));

    TRACE("Ublox GPS Setup done! (%p)\n", gps_ublox);
}

void Sensors::internalAdcCallback()
{
    LoggerService::getInstance()->log(internal_adc->getLastSample());
}

void Sensors::batteryVoltageCallback()
{
    // float v = battery_voltage->getLastSample().bat_voltage;
    // if (v < 10.0)
    // {
    //     LOG_WARN(log, "******* LOW BATTERY ******* \n Voltage = %.2f \n", v);
    // }

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
void Sensors::hilSensorsInit()
{
    HILTransceiver* simulator = HIL::getInstance()->simulator;

    hil_imu  = new HILImu(simulator, N_DATA_IMU);
    hil_baro = new HILBarometer(simulator, N_DATA_BARO);
    hil_gps  = new HILGps(simulator, N_DATA_GPS);

    SensorInfo info_imu("HILImu", HIL_IMU_PERIOD, bind(&Sensors::hilIMUCallback, this),
                        false, true);
    SensorInfo info_baro("HILBaro", HIL_BARO_PERIOD, bind(&Sensors::hilBaroCallback, this),
                         false, true);
    SensorInfo info_gps("HILGps", HIL_GPS_PERIOD, bind(&Sensors::hilGPSCallback, this),
                        false, true);

    sensors_map.emplace(std::make_pair(hil_imu, info_imu));
    sensors_map.emplace(std::make_pair(hil_baro, info_baro));
    sensors_map.emplace(std::make_pair(hil_gps, info_gps));

    TRACE("HIL Sensors setup done! \n");
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
    LoggerService::getInstance()->log(press_pitot->getLastSample());
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
    //     LOG_DEBUG_ASYNC(log.getChild("bmx160"),
    //                     "acc xyz: {:+.3f},{:+.3f},{:+.3f} gyro xyz: "
    //                     "{:+.3f},{:+.3f},{:+.3f}",
    //                     sample.accel_x, sample.accel_y, sample.accel_z,
    //                     sample.gyro_x, sample.gyro_y, sample.gyro_z);
    // }
}

void Sensors::magLISCallback()
{
    LoggerService::getInstance()->log(mag_lis3mdl->getLastSample());

    // static unsigned int downsample_ctr = 0;

    // if (downsample_ctr++ % 20 == 0)
    // {
    //     auto sample = mag_lis3mdl->getLastSample();
    //     LOG_DEBUG_ASYNC(log.getChild("lis3mdl"),
    //                     "mag xyzt: {:+.3f},{:+.3f},{:+.3f},{:+.3f}",
    //                     sample.mag_x, sample.mag_y, sample.mag_z,
    //                     sample.temp);
    // }
}

void Sensors::gpsUbloxCallback()
{
    /*UbloxGPSData d = gps_ublox->getLastSample();
    TRACE("%llu %d %f %f %u %f %f \n", d.gps_timestamp, d.fix, d.latitude,
          d.longitude, d.num_satellites, d.speed, d.track);*/

    LoggerService::getInstance()->log(gps_ublox->getLastSample());
}

#ifdef HARDWARE_IN_THE_LOOP
void Sensors::hilIMUCallback()
{
    LoggerService::getInstance()->log(hil_imu->getLastSample());

    // TRACE("HILImu callback \n");
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

}  // namespace DeathStackBoard