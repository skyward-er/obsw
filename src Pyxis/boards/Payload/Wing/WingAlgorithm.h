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

#include <Payload/Actuators/Actuators.h>
#include <Payload/Wing/WingAlgorithmData.h>
#include <algorithms/Algorithm.h>
#include <diagnostic/PrintLogger.h>
#include <logger/Logger.h>
#include <miosix.h>

namespace Payload
{
class WingAlgorithm : public Boardcore::Algorithm
{
public:
    /**
     * @brief Construct a new Wing Algorithm object
     *
     * @param servo1 The first servo
     * @param servo2 The second servo
     */
    WingAlgorithm(ServosList servo1, ServosList servo2);

    /**
     * @brief Method to initialize the algorithm
     *
     * @return true If the init process goes well
     * @return false If the init process doesn't go well
     */
    bool init() override;

    /**
     * @brief Set the Servos object
     *
     * @param servo1 The first algorithm servo
     * @param servo2 The second algorithm servo
     */
    void setServo(ServosList servo1, ServosList servo2);

    /**
     * @brief Add a step to the algorithm sequence
     *
     * @param step The step to add
     */
    void addStep(WingAlgorithmData step);

    /**
     * @brief This sets the reference timestamp for the algorithm
     */
    void begin();

    /**
     * @brief This method disabloes the algorithm
     */
    void end();

protected:
    // The actuators
    ServosList servo1, servo2;

    // Reference timestamp for relative algorithm timestamps
    uint64_t timeStart;

    // Procedure array to memorize all the steps needed to perform the algorithm
    std::vector<WingAlgorithmData> steps;

    // PrintLogger
    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("WingAlgorithm");

    // SD logger
    Boardcore::Logger* SDlogger = &Boardcore::Logger::getInstance();

    // This boolean is used to understand when to reset
    // the index where the algorithm has stopped.
    // In case of end call, we want to be able to perform
    // another time this algorithm starting from 0
    bool shouldReset = false;

    /**
     * @brief This method implements the algorithm step based on the current
     * timestamp
     */
    void step() override;
};
}  // namespace Payload