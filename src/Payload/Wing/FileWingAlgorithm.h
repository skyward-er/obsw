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
#include <Payload/Wing/WingAlgorithm.h>
#include <utils/CSVReader/CSVReader.h>

/**
 * @brief This class abstracts the concept of wing timestamp based
 * algorithm. These algorithms are stored in files (formatted in csv).
 * We use a CSV parser to properly parse the procedure and every step
 * we check if it is time to advance and in case actuate the step with
 * the two servos.
 *
 * Brief use case:
 *
 * Actuators::getInstance().enableServo(PARAFOIL_LEFT_SERVO);
 * Actuators::getInstance().enableServo(PARAFOIL_RIGHT_SERVO);
 *
 * WingAlgorithm algorithm(PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO, "Optional
 * File") algorithm.init();
 *
 * //In case of a non valid file/empty string you can add the steps
 * algorithm.addStep(WingAlgorithmData{timestamp, angle1, angle2});
 *
 * algorithm.begin();
 *
 * algorithm.update()...
 *
 * //End of algorithm
 *
 * algorithm.begin();
 *
 * algorithm.update()...
 */

namespace Payload
{

class FileWingAlgorithm : public WingAlgorithm
{
public:
    /**
     * @brief Construct a new Wing Algorithm object
     *
     * @param servo1 The first servo
     * @param servo2 The second servo
     * @param filename The csv file where all the operations are stored
     */
    FileWingAlgorithm(ServosList servo1, ServosList servo2,
                      const char* filename);

    /**
     * @brief This method parses the file and stores it into a std::vector
     *
     * @return true If the initialization finished correctly
     * @return false If something went wrong
     */
    bool init() override;

protected:
    /**
     * @brief CSV format file parser
     */
    Boardcore::CSVParser<WingAlgorithmData> parser;

    /**
     * @brief Indicates whether the current file of the algorithm is readable
     */
    bool fileValid = false;
};
}  // namespace Payload
