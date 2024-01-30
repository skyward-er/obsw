/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro, Federico Mandelli
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

#include <Payload/Configs/WingConfig.h>
#include <Payload/Wing/WingAlgorithm.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace Boardcore;
using namespace Payload::WingConfig;
using namespace Common;
namespace Payload
{
WingAlgorithm::WingAlgorithm(ServosList servo1, ServosList servo2)
{
    this->servo1 = servo1;
    this->servo2 = servo2;
    stepIndex    = 0;
    // Create the vector for algorithm data
    steps = std::vector<WingAlgorithmData>();
}

bool WingAlgorithm::init()
{
    return true;  // In this case the init is always true
}

void WingAlgorithm::setServo(ServosList servo1, ServosList servo2)
{
    this->servo1 = servo1;
    this->servo2 = servo2;
}

void WingAlgorithm::addStep(WingAlgorithmData step) { steps.push_back(step); }

void WingAlgorithm::begin()
{
    running     = true;
    shouldReset = true;

    // Set the reference timestamp
    timeStart = TimestampTimer::getTimestamp();
}

void WingAlgorithm::end()
{
    running = false;

    // Set the reference timestamp to 0
    timeStart = 0;
}

void WingAlgorithm::step()
{
    ModuleManager& modules    = ModuleManager::getInstance();
    uint64_t currentTimestamp = TimestampTimer::getTimestamp();

    if (shouldReset)
    {
        // If the algorithm has been stopped
        // i want to start from the beginning
        stepIndex   = 0;
        shouldReset = false;
    }

    if (stepIndex >= steps.size())
    {
        LOG_INFO(logger, "Algorithm end {:d} >= {:d}", stepIndex, steps.size());
        // End the procedure so it won't be executed
        // Set the index to 0 in case of another future execution
        stepIndex = 0;
        // Terminate here
        EventBroker::getInstance().post(WING_ALGORITHM_ENDED, TOPIC_WING);
        return;
    }

    if (currentTimestamp - timeStart >= steps[stepIndex].timestamp)
    {
        // I need to execute the current step
        modules.get<Actuators>()->setServoAngle(servo1,
                                                steps[stepIndex].servo1Angle);
        modules.get<Actuators>()->setServoAngle(servo2,
                                                steps[stepIndex].servo2Angle);

        // Log the data setting the timestamp to the absolute one
        WingAlgorithmData data;
        data.timestamp   = TimestampTimer::getTimestamp();
        data.servo1Angle = steps[stepIndex].servo1Angle;
        data.servo2Angle = steps[stepIndex].servo2Angle;

        // After copy i can log the actual step
        SDlogger->log(data);

        // finally increment the stepIndex
        stepIndex++;

        LOG_INFO(logger, "Step");
    }
}

}  // namespace Payload
