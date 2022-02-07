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

#include <utils/CSVReader/CSVReader.h>
#include <common/Algorithm.h>
#include <Wing/WingServo.h>

/**
 * @brief This class abstracts the concept of wing timestamp based
 * algorithm. These algorithms are stored in files (formatted in csv).
 * We use a CSV parser to properly parse the procedure and every step
 * we check if it is time to advance and in case actuate teh step with
 * the two servos.
 * 
 * Brief use case:
 * WingServo servo1...
 * WingServo servo2...
 * 
 * servo1.enable();
 * servo2.enable();
 * 
 * WingAlgorithm algorithm(&servo1, &servo2, "Optional File")
 * algorithm.init();
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

using namespace Boardcore;

namespace ParafoilTestDev
{
    struct WingAlgorithmData
    {
        uint64_t timestamp; //First timestamp is 0 (in microseconds)
        float servo1Angle;  //degrees
        float servo2Angle;  //degrees
    };

    class WingAlgorithm : public Algorithm
    {
    public:

        /**
         * @brief Construct a new Wing Algorithm object
         * 
         * @param servo1 The first servo
         * @param servo2 The second servo
         * @param filename The csv file where all the operations are stored
         */
        WingAlgorithm(ServoInterface* servo1, ServoInterface* servo2, const char* filename);

        /**
         * @brief Construct a new Wing Algorithm object
         * 
         * @param filename The csv file where all the operations are stored
         */
        WingAlgorithm(const char* filename);

        /**
         * @brief This method parses the file and stores it into a std::vector
         * 
         * @return true If the initialization finished correctly
         * @return false If something went wrong
         */
        bool init() override;

        /**
         * @brief Set the Servos objects
         * @param servo1 The first algorithm servo
         * @param servo2 The second algorithm servo
         */
        void setServo(ServoInterface* servo1, ServoInterface* servo2);

        /**
         * @brief Adds manually the step in case of fast debug needs
         * 
         * @param step The data that describes a step(timestamp, servo1 angle, servo2 angle)
         */
        void addStep(WingAlgorithmData step);

        /**
         * @brief This sets the reference timestamp for the algorithm
         */
        void begin();

        /**
         * @brief This disables the algorithm
         */
        void end();

    protected:
        /**
         * @brief This method implements the algorithm step based on the current timestamp
         */
        void step() override;

        //Actuators
        ServoInterface* servo1;
        ServoInterface* servo2;
        //Offset timestamp
        uint64_t timeStart;
        //Procedure
        std::vector<WingAlgorithmData> steps;
        //File parser
        CSVParser<WingAlgorithmData> parser;
        bool fileValid      = false;
        //This boolean is used to understand when to reset
        //the index where the algorithm has stopped.
        //In case of end call, we want to be able to perform
        //another time this algorithm starting from 0
        bool shouldReset    = false;
    };
}