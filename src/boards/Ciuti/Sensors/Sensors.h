/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <Singleton.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/adc/InternalADC.h>
#include <sensors/LIS331HH/LIS331HH.h>
#include <sensors/SensorManager.h>

namespace Ciuti
{

class Sensors : public Boardcore::Singleton<Sensors>
{
    friend class Boardcore::Singleton<Sensors>;

public:
    bool start();

    Boardcore::InternalADCData getInternalADCLastSample(
        Boardcore::InternalADC::Channel channel);

    Boardcore::LIS331HHData getLIS331HHLastSample();

private:
    Sensors();
    ~Sensors();

    void internalAdcInit();
    void lis331hhInit();

    Boardcore::InternalADC *internalAdc = nullptr;
    Boardcore::LIS331HH *lis331hh       = nullptr;

    Boardcore::SensorManager *sensorManager = nullptr;

    Boardcore::SensorManager::SensorMap_t sensorsMap;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");
};

}  // namespace Ciuti