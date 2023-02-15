/* Copyright (c) 2022 Skyward Experimental Rocketry
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

#pragma once

#include <Singleton.h>
#include <diagnostic/PrintLogger.h>
#include <sensors/LIS331HH/LIS331HHData.h>
#include <utils/MovingAverage.h>

#include "UprightDetectorConfig.h"

namespace Ciuti
{

class UprightDetector
{
public:
    UprightDetector() {}

    void update(float axis);
    bool isUpright() { return upright; }

private:
    bool upright = false;
    int count    = 0;
    Boardcore::MovingAverage<float, UprightDetectorConfig::MEAN_SAMPLES>
        filtered;
};

class UprightDetectorController
    : public Boardcore::Singleton<UprightDetectorController>
{
    friend class Boardcore::Singleton<UprightDetectorController>;

public:
    void start();

private:
    UprightDetectorController() {}

    void update();
    void trigger();

    bool fired = false;

    UprightDetector algo;

    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("ciuti.uprightdetector");
};

}  // namespace Ciuti