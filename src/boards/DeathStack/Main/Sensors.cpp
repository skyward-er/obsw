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

Sensors::Sensors(SPIBusInterface& spi1_bus) : spi1_bus(spi1_bus)
{
    // Pressure sensors
    pressDigiInit();
    ADS1118Init();
    pressPitotInit();
    pressDPLVaneInit();
    pressStaticInit();
    imuBMXinit();
    magLISinit();
    sensor_manager = new SensorManager(sensors_map);
}

Sensors::~Sensors()
{
    delete press_digital;
    delete adc_ads1118;
    delete press_pitot;
    delete press_dpl_vane;
    delete press_static_port;
    delete imu_bmx160;

    sensor_manager->stop();
    delete sensor_manager;
}

void Sensors::start()
{
    GpioPin int_pin = miosix::sensors::bmx160::intr::getPin();

    enableExternalInterrupt(int_pin.getPort(), int_pin.getNumber(),
                            InterruptTrigger::FALLING_EDGE);

    sensor_manager->start();
}

void Sensors::pressDigiInit()
{
    SPIBusConfig spi_cfg{};
    spi_cfg.clock_div = SPIClockDivider::DIV16;

    press_digital = new MS580301BA07(
        spi1_bus, miosix::sensors::ms5803::cs::getPin(), spi_cfg);

    SensorInfo info(SAMPLE_PERIOD_PRESS_DIGITAL,
                    bind(&Sensors::pressDigiCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_digital, info));

    LOG_INFO(log, "MS5803 pressure sensor setup done! ({:p})",
             fmt::ptr(press_digital));
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

    SensorInfo info(SAMPLE_PERIOD_ADC_ADS1118,
                    bind(&Sensors::ADS1118Callback, this), false, true);
    sensors_map.emplace(std::make_pair(adc_ads1118, info));

    LOG_INFO(log, "ADS1118 setup done! ({:p})", fmt::ptr(adc_ads1118));
}

void Sensors::pressPitotInit()
{

    function<ADCData()> voltage_fun(
        bind(&ADS1118::getVoltage, adc_ads1118, ADC_CH_PITOT_PORT));
    press_pitot = new SSCDRRN015PDA(voltage_fun, REFERENCE_VOLTAGE);

    SensorInfo info(SAMPLE_PERIOD_PRESS_PITOT,
                    bind(&Sensors::pressPitotCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_pitot, info));

    LOG_INFO(log, "Pitot pressure sensor setup done! ({:p})",
             fmt::ptr(press_pitot));
}

void Sensors::pressDPLVaneInit()
{

    function<ADCData()> voltage_fun(
        bind(&ADS1118::getVoltage, adc_ads1118, ADC_CH_DPL_PORT));
    press_dpl_vane = new SSCDANN030PAA(voltage_fun, REFERENCE_VOLTAGE);

    SensorInfo info(SAMPLE_PERIOD_PRESS_DPL,
                    bind(&Sensors::pressDPLVaneCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_dpl_vane, info));

    LOG_INFO(log, "DPL pressure sensor setup done! ({:p})",
             fmt::ptr(press_dpl_vane));
}

void Sensors::pressStaticInit()
{

    function<ADCData()> voltage_fun(
        bind(&ADS1118::getVoltage, adc_ads1118, ADC_CH_STATIC_PORT));
    press_static_port = new MPXHZ6130A(voltage_fun, REFERENCE_VOLTAGE);

    SensorInfo info(SAMPLE_PERIOD_PRESS_STATIC,
                    bind(&Sensors::pressStaticCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_static_port, info));

    LOG_INFO(log, "Static pressure sensor setup done! ({:p})",
             fmt::ptr(press_static_port));
}

void Sensors::imuBMXinit()
{
    SPIBusConfig spi_cfg = ADS1118::getDefaultSPIConfig();
    spi_cfg.clock_div    = SPIClockDivider::DIV32;

    BMX160Config bmx_config;
    bmx_config.fifo_mode      = BMX160Config::FifoMode::HEADER;
    bmx_config.fifo_watermark = IMU_BMX_FIFO_WATERMARK;
    bmx_config.fifo_int       = BMX160Config::FifoInt::PIN_INT1;

    bmx_config.temp_divider = 1;

    bmx_config.acc_range = BMX160Config::AccRange::G_16;
    bmx_config.gyr_range = BMX160Config::GyrRange::DEG_125;

    bmx_config.acc_odr = IMU_BMX_ACC_GYRO_FS_ENUM;
    bmx_config.gyr_odr = IMU_BMX_ACC_GYRO_FS_ENUM;
    bmx_config.mag_odr = IMU_BMX_MAG_FS_ENUM;

    imu_bmx160 = new BMX160(spi1_bus, miosix::sensors::bmx160::cs::getPin(),
                            bmx_config, spi_cfg);

    SensorInfo info(SAMPLE_PERIOD_IMU_BMX, bind(&Sensors::imuBMXCallback, this),
                    false, true);

    sensors_map.emplace(std::make_pair(imu_bmx160, info));

    LOG_INFO(log, "BMX160 Setup done! ({:p})", fmt::ptr(imu_bmx160));
}

void Sensors::magLISinit()
{
    SPIBusConfig busConfig;
    busConfig.clock_div = SPIClockDivider::DIV32;

    LIS3MDL::Config config;
    config.odr                = MAG_LIS_FS_ENUM;
    config.scale              = MAG_LIS_FULLSCALE;
    config.temperatureDivider = 1;

    mag_lis3mdl = new LIS3MDL(spi1_bus, miosix::sensors::lis3mdl::cs::getPin(),
                              busConfig, config);

    SensorInfo info(SAMPLE_PERIOD_MAG_LIS, bind(&Sensors::magLISCallback, this),
                    false, true);

    sensors_map.emplace(std::make_pair(mag_lis3mdl, info));

    LOG_INFO(log, "LIS3MDL Setup done! ({:p})", fmt::ptr(mag_lis3mdl));
}

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

}  // namespace DeathStackBoard