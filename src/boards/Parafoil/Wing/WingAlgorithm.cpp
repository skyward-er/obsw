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

#include <Parafoil/Actuators/Actuators.h>
#include <Parafoil/Wing/WingAlgorithm.h>
#include <drivers/timer/TimestampTimer.h>

using namespace Boardcore;

namespace Parafoil
{

std::istream& operator>>(std::istream& input, WingAlgorithmData& data)
{
    input >> data.timestamp;
    input.ignore(1, ',');
    input >> data.servo1Angle;
    input.ignore(1, ',');
    input >> data.servo2Angle;
    return input;
}

WingAlgorithm::WingAlgorithm(const char* filename) : parser(filename) {}

bool WingAlgorithm::init()
{
    // Returns a std::vector which contains
    // all the csv parsed with the data structure in mind
    steps = parser.collect();

    // Return if the size collected is greater than 0
    fileValid = steps.size() > 0;

    // Communicate it via serial
    if (fileValid)
    {
        LOG_INFO(logger, "File valid");
    }

    return fileValid;
}

void WingAlgorithm::addStep(WingAlgorithmData step)
{
    // I do it if and only if the file is invalid, because
    // i don't want to mess up with the timestamp order
    if (!fileValid)
    {
        // Add it to the std::vector at the end
        steps.push_back(step);
    }
}

void WingAlgorithm::begin()
{
    running     = true;
    shouldReset = true;
    // Set the current timestamp
    timeStart = TimestampTimer::getTimestamp();
}

void WingAlgorithm::end()
{
    running = false;
    // Set the offset timestamp to 0
    timeStart = 0;
}

void WingAlgorithm::step()
{
    // Variable to remember what is the step that has to be done
    static unsigned int stepIndex = 0;
    uint64_t currentTimestamp     = TimestampTimer::getTimestamp();

    if (shouldReset)
    {
        // If the algorithm has been stopped i want to start from the beginning
        stepIndex   = 0;
        shouldReset = false;
    }

    if (stepIndex >= steps.size())
    {
        // End the procedure so it won't be executed
        end();

        // Set the index to 0 in case of another future execution
        stepIndex = 0;

        return;
    }

    if (currentTimestamp - timeStart >= steps[stepIndex].timestamp)
    {
        Actuators::getInstance().setServo(PARAFOIL_SERVO1,
                                          steps[stepIndex].servo1Angle);
        Actuators::getInstance().setServo(PARAFOIL_SERVO2,
                                          steps[stepIndex].servo2Angle);

        WingAlgorithmData data;
        data.timestamp   = TimestampTimer::getTimestamp();
        data.servo1Angle = steps[stepIndex].servo1Angle;
        data.servo2Angle = steps[stepIndex].servo2Angle;
        Logger::getInstance().log(data);

        stepIndex++;
    }
}

}  // namespace Parafoil
