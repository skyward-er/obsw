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

using std::bind;
using std::function;

namespace DeathStackBoard
{

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
    SPIBusConfig spi_cfg{};
    spi_cfg.clock_div = SPIClockDivider::DIV32;
    spi_cfg.mode      = SPIMode::MODE1;

    adc_ads1118 = new ADS1118(spi1_bus, miosix::sensors::ads1118::cs::getPin(),
                              ADS1118::ADS1118_DEFAULT_CONFIG, spi_cfg);

    SensorInfo info(50, bind(&Sensors::ADS1118Callback, this), false, true);

    sensors_map.emplace(std::make_pair(adc_ads1118, info));
}

void Sensors::ADS1118Callback()
{
    LoggerService::getInstance()->log(adc_ads1118->getLastSample());
}

void Sensors::pressPitotInit()
{
    constexpr ADS1118::ADS1118Mux channel = ADS1118::ADS1118Mux::MUX_AIN2_GND;

    function<ADCData()> voltage_fun(bind(&ADS1118::getVoltage, adc_ads1118, channel));
    press_pitot = new SSCDRRN015PDA(voltage_fun);

    SensorInfo info(50, bind(&Sensors::pressPitotCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_pitot, info));
}

void Sensors::pressPitotCallback()
{
    LoggerService::getInstance()->log(adc_ads1118->getLastSample());
}

void Sensors::pressDPLVaneInit()
{
    constexpr ADS1118::ADS1118Mux channel = ADS1118::ADS1118Mux::MUX_AIN2_GND;

    function<ADCData()> voltage_fun(bind(&ADS1118::getVoltage, adc_ads1118, channel));
    press_dpl_vane = new SSCDANN030PAA(voltage_fun);

    SensorInfo info(50, bind(&Sensors::pressDPLVaneCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_dpl_vane, info));
}

void Sensors::pressDPLVaneCallback()
{
    LoggerService::getInstance()->log(press_dpl_vane->getLastSample());
}

void Sensors::pressStaticInit()
{
    constexpr ADS1118::ADS1118Mux channel = ADS1118::ADS1118Mux::MUX_AIN2_GND;

    function<ADCData()> voltage_fun(bind(&ADS1118::getVoltage, adc_ads1118, channel));
    press_static_port = new MPXHZ6130A(voltage_fun);

    SensorInfo info(50, bind(&Sensors::pressDPLVaneCallback, this), false, true);

    sensors_map.emplace(std::make_pair(press_static_port, info));
}

void Sensors::pressStaticCallback()
{
    LoggerService::getInstance()->log(press_static_port->getLastSample());
}


}  // namespace DeathStackBoard