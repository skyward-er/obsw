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

#include <interfaces-impl/hwmapping.h>

#include <functional>
#include <utility>

#include "LoggerService/LoggerService.h"

#include "configs/SensorManagerConfig.h"
using std::bind;
using std::function;

namespace DeathStackBoard
{
using namespace SensorConfigs;

Sensors::Sensors(SPIBusInterface& spi1_bus) : spi1_bus(spi1_bus)
{
    pressDigiInit();
    ADS1118Init();
    pressPitotInit();
    pressDPLVaneInit();
    pressStaticInit();

    sensor_manager = new SensorManager(sensors_map);
}

Sensors::~Sensors()
{
    delete press_digital;
    delete adc_ads1118;
    delete press_pitot;
    delete press_dpl_vane;
    delete press_static_port;

    sensor_manager->stop();
    delete sensor_manager;
}

void Sensors::start() { sensor_manager->start(); }

void Sensors::pressDigiInit()
{
    SPIBusConfig spi_cfg{};
    spi_cfg.clock_div = SPIClockDivider::DIV16;

    press_digital = new MS580301BA07(
        spi1_bus, miosix::sensors::ms5803::cs::getPin(), spi_cfg);

    SensorInfo info(50, bind(&Sensors::pressDigiCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_digital, info));
}

void Sensors::pressDigiCallback()
{
    LoggerService::getInstance()->log(press_digital->getLastSample());
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
    SensorInfo info(150, bind(&Sensors::ADS1118Callback, this), false, true);
    sensors_map.emplace(std::make_pair(adc_ads1118, info));
}

void Sensors::ADS1118Callback()
{
    LoggerService::getInstance()->log(adc_ads1118->getLastSample());
}

void Sensors::pressPitotInit()
{

    function<ADCData()> voltage_fun(
        bind(&ADS1118::getVoltage, adc_ads1118, ADC_CH_PITOT_PORT));
    press_pitot = new SSCDRRN015PDA(voltage_fun, REFERENCE_VOLTAGE);

    SensorInfo info(50, bind(&Sensors::pressPitotCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_pitot, info));
}

void Sensors::pressPitotCallback()
{

    // TRACE("PITOT Callback: %f\n", press_pitot->getLastSample().press);
    LoggerService::getInstance()->log(press_pitot->getLastSample());
}

void Sensors::pressDPLVaneInit()
{

    function<ADCData()> voltage_fun(
        bind(&ADS1118::getVoltage, adc_ads1118, ADC_CH_DPL_PORT));
    press_dpl_vane = new SSCDANN030PAA(voltage_fun, REFERENCE_VOLTAGE);

    SensorInfo info(50, bind(&Sensors::pressDPLVaneCallback, this), false,
                    true);

    sensors_map.emplace(std::make_pair(press_dpl_vane, info));
}

void Sensors::pressDPLVaneCallback()
{
    LoggerService::getInstance()->log(press_dpl_vane->getLastSample());
}

void Sensors::pressStaticInit()
{

    function<ADCData()> voltage_fun(
        bind(&ADS1118::getVoltage, adc_ads1118, ADC_CH_STATIC_PORT));
    press_static_port = new MPXHZ6130A(voltage_fun, REFERENCE_VOLTAGE);

    SensorInfo info(50, bind(&Sensors::pressStaticCallback, this), false,
                    true);

    sensors_map.emplace(std::make_pair(press_static_port, info));
}

void Sensors::pressStaticCallback()
{

    // TRACE("Static Callback: %f\n", press_static_port->getLastSample().press);

    LoggerService::getInstance()->log(press_static_port->getLastSample());
}

}  // namespace DeathStackBoard