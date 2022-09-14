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

#include "UprightDetector.h"

#include <Ciuti/BoardScheduler.h>
#include <Ciuti/Configs/SensorsConfig.h>
#include <Ciuti/Sensors/Sensors.h>
#include <utils/Debug.h>

#include <cmath>

/*
% Matlab algorithm
% Author: Marco Marchesi
testlog = ciutilog;
testlog.timestamp = testlog.timestamp*1e-6;
angle_trigger = deg2rad(75);
% maybe transform in radians% flags
flagDetect = false;
counter = 0;
%
detectTime = 5; %[s]
sensorFrequency = 50; %[Hz]
N_detectSamples = detectTime * sensorFrequency;
z_filter = movmean(testlog.z,50);
maxZ = max(abs(z_filter));
offset = 0.20;
z_filter = z_filter + offset;
threshold = sin(angle_trigger);
for i = 1:length(z_filter)
    if abs(z_filter(i)) > threshold
        counter = counter + 1;
    else
        counter = 0;
    end
    if counter > N_detectSamples
        flagDetect = true;
    end
    detection_vec(i) = flagDetect;
end
*/

using namespace Boardcore;

namespace Ciuti
{

void UprightDetector::update(float axis)
{
    filtered.push(axis);

    if (std::abs(filtered.getAverage()) > UprightDetectorConfig::THRESHOLD)
    {
        count++;
    }
    else
    {
        count = 0;
    }

    upright = count > UprightDetectorConfig::DETECT_SAMPLES;
}

void UprightDetectorController::start()
{
    BoardScheduler::getInstance().getScheduler().addTask(
        [=]() { this->update(); }, UprightDetectorConfig::ALGO_PERIOD);
}

void UprightDetectorController::update()
{
    auto sample = Sensors::getInstance().getLIS331HHLastSample();

    if (!fired)
    {
        algo.update(sample.accelerationZ -
                    Ciuti::SensorsConfig::Z_AXIS_OFFSET_LIS331HH);

        if (algo.isUpright())
        {
            trigger();
            fired = true;
        }
    }
}

void UprightDetectorController::trigger()
{
    LOG_INFO(logger, "Upright triggered!");

    Logger::getInstance().resetStats();
    Logger::getInstance().start();
}

}  // namespace Ciuti
