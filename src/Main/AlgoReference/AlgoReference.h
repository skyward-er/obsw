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

#pragma once

#include <Main/Sensors/Sensors.h>
#include <algorithms/ReferenceValues.h>
#include <miosix.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <chrono>

namespace Main
{
/**
 * An interface class for modules interested in being notified when the
 * reference values change.
 */
struct ReferenceSubscriber
{
    virtual void onReferenceChanged(const Boardcore::ReferenceValues& ref) = 0;
};

class AlgoReference : public Boardcore::InjectableWithDeps<Sensors>
{
public:
    AlgoReference();

    void calibrate();

    Boardcore::ReferenceValues getReferenceValues();

    void subscribeReferenceChanges(ReferenceSubscriber* sub)
    {
        refSubscribers.push_back(sub);
    }

    void setReferenceAltitude(float altitude);
    void setReferenceTemperature(float temperature);
    void setReferenceCoordinates(float latitude, float longitude);

    /**
     * @brief Compute time since liftoff accounting for detection delays.
     */
    std::chrono::milliseconds computeTimeSinceLiftoff(
        std::chrono::milliseconds duration);

    void setRampPinDetectionDelay(std::chrono::milliseconds delay);

private:
    void notifyReferenceChanged();

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("reference");

    std::atomic<std::chrono::milliseconds> rampPinDetectionDelay = {};

    std::vector<ReferenceSubscriber*> refSubscribers;

    miosix::FastMutex referenceMutex;
    Boardcore::ReferenceValues reference;
};

}  // namespace Main
