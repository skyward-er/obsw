/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio
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

#pragma once

#include <diagnostic/PrintLogger.h>
#include <sensors/ADS1118/ADS1118.h>
#include <sensors/SensorManager.h>
#include <sensors/analog/pressure/honeywell/SSCDRRN015PDA.h>

namespace WindGallery
{

class Sensors : public Boardcore::Singleton<Sensors>
{
    friend Boardcore::Singleton<Sensors>;

public:
    Boardcore::ADS1118 *ads1118             = nullptr;
    Boardcore::SSCDRRN015PDA *pitotPressure = nullptr;

    bool start();

private:
    Sensors();

    void ads1118Init();

    void pitotPressureInit();
    void pitotPressureCallback();

    Boardcore::SensorManager *sensorManager = nullptr;

    Boardcore::SensorManager::SensorMap_t sensorsMap;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");
};

}  // namespace WindGallery
