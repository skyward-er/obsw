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

#include <Wing/WingAlgorithm.h>
#include <drivers/timer/TimestampTimer.h>

namespace ParafoilTestDev
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

    /**
     * PUBLIC METHODS
     */
    WingAlgorithm::WingAlgorithm(WingServo* servo1, WingServo* servo2, const char* filename)
                 : parser(filename)
    {
        this -> servo1 = servo1;
        this -> servo2 = servo2;
    }

    bool WingAlgorithm::init()
    {
        //Returns a std::vector which contains
        //all the csv parsed with the data structure in mind
        steps = parser.collect();
        
        //Return if the size collected is greater than 0
        fileValid = steps.size() > 0;
        return fileValid;
    }

    void WingAlgorithm::addStep(WingAlgorithmData step)
    {
        //I do it if and only if the file is invalid, because 
        //i don't want to mess up with the timestamp order
        if(!fileValid)
        {
            //Add it to the std::vector at the end
            steps.push_back(step);
        }
    }

    void WingAlgorithm::begin()
    {
        running = true;
        //Set the current timestamp
        timeStart = TimestampTimer::getInstance().getTimestamp();
    }

    void WingAlgorithm::end()
    {
        running = false;
        //Set the offset timestamp to 0
        timeStart = 0;
    }

    /**
     * PROTECTED METHODS
     */
    void WingAlgorithm::step()
    {
        //Variable to remember the what is the stab that has to be done
        static unsigned int stepIndex = 0;
        uint64_t currentTimestamp = TimestampTimer::getInstance().getTimestamp();
        
        

        if(stepIndex >= steps.size())
        {
            //End the procedure so it won't be executed
            end();
            //Set the index to 0 in case of another future execution
            stepIndex = 0;
            //Terminate here
            return;
        }

        if(currentTimestamp - timeStart >= steps[stepIndex].timestamp)
        {
            //I need to execute the current step
            servo1->set(steps[stepIndex].servo1Angle);
            servo2->set(steps[stepIndex].servo1Angle);

            //finally increment the stepIndex
            stepIndex++;
        }
    }
}