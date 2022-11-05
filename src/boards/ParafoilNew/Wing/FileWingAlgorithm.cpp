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

#include <ParafoilNew/Wing/FileWingAlgorithm.h>
#include <drivers/timer/TimestampTimer.h>

using namespace Boardcore;

namespace Payload
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

FileWingAlgorithm::FileWingAlgorithm(ServosList servo1, ServosList servo2,
                                     const char* filename)
    : WingAlgorithm(servo1, servo2), parser(filename)
{
    setServo(servo1, servo2);
}

bool FileWingAlgorithm::init()
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

    // Close the file
    parser.close();

    return fileValid;
}

}  // namespace Payload
