/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Mozzarelli
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <Common.h>
#include <sensors/Sensor.h>
#include <tests/catch/ada/test-ada-data.h>
#include <random>

namespace DeathStackBoard
{
    class MockPressureSensor : public ::PressureSensor
    {
        public:
        bool init()
        {
            return true;
        }

        bool selfTest()
        {
            return true;
        }

        bool onSimpleUpdate()
        {
            if (before_liftoff)
            {
                 mLastPressure = addNoise(SIMULATED_PRESSURE[0]);
            }
            else
            {
                mLastPressure = addNoise(SIMULATED_PRESSURE[i]);
            }
            i++;
            return true;
        }

        bool before_liftoff = true;
        
        private:
        int i = 0;      // Last index
        std::default_random_engine generator{12345};
        std::normal_distribution<float> distribution{0.0f,50.0f};

        float addNoise(float sample)
        {
            return quantization(sample+distribution(generator));
        }

        float quantization(float sample)
        {
            return round(sample/30)*30;
        }
    };
}
