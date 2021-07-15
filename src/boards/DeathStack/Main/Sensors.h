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

#pragma once

#include <map>

#include <drivers/spi/SPIBusInterface.h>
#include <sensors/SensorManager.h>

#include <drivers/adc/ADS1118/ADS1118.h>
#include <sensors/analog/pressure/honeywell/SSCDRRN015PDA.h>
#include <sensors/analog/pressure/honeywell/SSCDANN030PAA.h>

#include <sensors/analog/pressure/MPXHZ6130A/MPXHZ6130A.h>
#include <sensors/MS580301BA07/MS580301BA07.h>

namespace DeathStackBoard
{

/**
 * @brief Initializes all the sensors on the death stack
 *
 */
class Sensors
{
public:
    SensorManager* sensor_manager;

    MS580301BA07* press_digital;

    ADS1118* adc_ads1118;
    SSCDRRN015PDA* press_pitot;
    SSCDANN030PAA* press_dpl_vane;
    MPXHZ6130A* press_static_port;

    Sensors(SPIBusInterface& spi1_bus);

    ~Sensors();

    void start();

private:
    void pressDigiInit();
    void pressDigiCallback();

    void ADS1118Init();
    void ADS1118Callback();

    void pressPitotInit();
    void pressPitotCallback();

    void pressDPLVaneInit();
    void pressDPLVaneCallback();

    void pressStaticInit();
    void pressStaticCallback();

    SPIBusInterface& spi1_bus;

    SensorManager::SensorMap_t sensors_map;
};

}