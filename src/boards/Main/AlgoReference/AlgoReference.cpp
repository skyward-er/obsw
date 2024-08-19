/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include "AlgoReference.h"

#include <Main/Configs/ReferenceConfig.h>
#include <common/ReferenceConfig.h>
#include <utils/AeroUtils/AeroUtils.h>

using namespace Main;
using namespace Boardcore;
using namespace miosix;
using namespace Common;

AlgoReference::AlgoReference()
    : reference{ReferenceConfig::defaultReferenceValues}
{
}

void AlgoReference::calibrate()
{
    Sensors* sensors = getModule<Sensors>();

    float baroAcc = 0.0f;

    for (int i = 0; i < Config::Reference::CALIBRATION_SAMPLES_COUNT; i++)
    {
        PressureData baro = sensors->getAtmosPressureLastSample();
        baroAcc += baro.pressure;

        Thread::sleep(Config::Reference::CALIBRATION_SLEEP_TIME);
    }

    baroAcc /= Config::Reference::CALIBRATION_SAMPLES_COUNT;

    Lock<FastMutex> lock{referenceMutex};

    // Compute reference altitude
    reference.refPressure = baroAcc;
    reference.refAltitude = Aeroutils::relAltitude(
        reference.refPressure, reference.mslPressure, reference.mslTemperature);

    // Also updated the reference with the GPS if we have fix
    UBXGPSData gps = sensors->getUBXGPSLastSample();
    if (gps.fix == 3)
    {
        // We do not use the GPS altitude because it sucks
        reference.refLatitude  = gps.latitude;
        reference.refLongitude = gps.longitude;
    }

    sdLogger.log(reference);
}

ReferenceValues AlgoReference::getReferenceValues()
{
    Lock<FastMutex> lock{referenceMutex};
    return reference;
}