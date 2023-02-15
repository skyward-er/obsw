/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <sensors/BME280/BME280.h>
#include <sensors/MPU9250/MPU9250.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSerial.h>

namespace Parafoil
{

class Sensors : public Boardcore::Singleton<Sensors>
{
    friend class Boardcore::Singleton<Sensors>;

public:
    bool start();

    bool isStarted();

    Boardcore::MPU9250Data getMpu9250LastSample();
    Boardcore::UBXGPSData getUbxGpsLastSample();
    Boardcore::BME280Data getBme280LastSample();

    void calibrate();

    std::map<string, bool> getSensorsState();

private:
    Sensors();

    ~Sensors();

    void mpu9250init();

    void ubxGpsInit();
    void ubxGpsCallback();

    void bme280init();
    void bme280Callback();

    Boardcore::MPU9250* mpu9250;
    Boardcore::UBXGPSSerial* ubxGps;
    Boardcore::BME280* bme280;

    bool needsCalibration = false;

    Boardcore::SensorManager* sensorManager = nullptr;

    Boardcore::SensorManager::SensorMap_t sensorsMap;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");
};

}  // namespace Parafoil
