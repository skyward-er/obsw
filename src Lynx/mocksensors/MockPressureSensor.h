/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Mozzarelli
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

#include <mocksensors/MockSensorsData.h>
//#include <mocksensors/lynx_flight_data/lynx_press_data.h>
#include <mocksensors/lynx_flight_data/lynx_pressure_static_data.h>
#include <sensors/Sensor.h>

#include <random>

namespace DeathStackBoard
{

class MockPressureSensor : public Boardcore::Sensor<MockPressureData>
{
public:
    MockPressureSensor(bool with_noise_ = false) : with_noise(with_noise_) {}

    bool init() override { return true; }

    bool selfTest() override { return true; }

    MockPressureData sampleImpl() override
    {
        MockPressureData data;

        data.pressureTimestamp = Boardcore::TimestampTimer::getTimestamp();

        if (before_liftoff)
        {
            data.pressure = PRESSURE_STATIC_DATA[0];
        }
        else
        {
            if (i < PRESSURE_STATIC_DATA_SIZE)
            {
                data.pressure = PRESSURE_STATIC_DATA[i++];
            }
            else
            {
                data.pressure =
                    PRESSURE_STATIC_DATA[PRESSURE_STATIC_DATA_SIZE - 1];
            }
        }

        if (with_noise)
        {
            data.pressure = addNoise(data.pressure);
        }

        data.pressure = movingAverage(data.pressure);

        return data;
    }

    void signalLiftoff() { before_liftoff = false; }

private:
    bool with_noise;
    volatile bool before_liftoff = true;
    volatile unsigned int i      = 0;  // Last index
    std::default_random_engine generator{1234567};
    std::normal_distribution<float> distribution{0.0f, 5.0f};

    // moving average
    const float moving_avg_coeff = 0.95;
    float accumulator            = 0.0;

    float addNoise(float sample)
    {
        return quantization(sample + distribution(generator));
    }

    float quantization(float sample) { return round(sample / 30.0) * 30.0; }

    float movingAverage(float new_value)
    {
        accumulator = (moving_avg_coeff * new_value) +
                      (1.0 - moving_avg_coeff) * accumulator;
        return accumulator;
    }
};

}  // namespace DeathStackBoard
