/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Vincenzo Santomarco
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

#include <miosix.h>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#define private public

#include <AirBrakes/AirBrakesAlgorithm.h>
#include <AirBrakes/AirBrakesServo.h>

#include "drivers/HardwareTimer.h"
#include "sensors/Sensor.h"
#include "test_data.h"

using namespace DeathStackBoard;
using namespace std;
class MockActuator : public AirBrakesServo
{
public:
    MockActuator() {}
    // void setPosition(float alpha) { lastAlpha = round(alpha * 180 / PI); }
    float error(float alphaTest)
    {
        int error = abs(alphaTest - currentPosition);
        printf("Error: %d\n", error);
        return error;
    }

    // private:
    // int lastAlpha;
};

template <typename T>
class MockSensor : public Sensor<T>
{
private:
    int ts = 0;

protected:
    T sampleImpl() override
    {
        input_t input = DATA[ts].input;
        T sampled;

        sampled.z         = input.z;
        sampled.vz        = input.vz;
        sampled.vMod      = input.vMod;
        sampled.timestamp = ts;

        ts++;

        return sampled;
    }

    bool init() override { return true; }

    bool selfTest() override { return true; }
};

class InputClass
{
public:
    float z;
    float vz;
    float vMod;
    uint64_t timestamp;
};

MockActuator mockActuator;
MockSensor<InputClass> mockSensor;
AirBrakesControlAlgorithm<InputClass> algorithm(mockSensor, &mockActuator);

int main()
{
    {
        miosix::FastInterruptDisableLock dLock;

        // Enable TIM5 peripheral clocks
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    }

    HardwareTimer<uint32_t> hrclock(
        TIM5,
        TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1));
    hrclock.setPrescaler(127);
    hrclock.start();

    mockSensor.sample();

    uint32_t t1 = hrclock.tick();
    algorithm.begin();
    uint32_t t2 = hrclock.tick();

    cout << "z: " << mockSensor.getLastSample().z
         << "\tvz: " << mockSensor.getLastSample().vz
         << "\tvMod: " << mockSensor.getLastSample().vMod
         << "\tservo: " << mockActuator.getCurrentPosition() << "\n";

    float startTime   = hrclock.toMilliSeconds(t2 - t1);
    float stepTime    = 0;
    int maxAlphaError = 0;

    for (int i = 1; i < LEN_TEST; i++)
    {
        mockSensor.sample();
        t1 = hrclock.tick();
        algorithm.update();
        t2 = hrclock.tick();

        int error = mockActuator.error(DATA[i].output.alpha);

        if (error > maxAlphaError)
        {
            maxAlphaError = error;
        }

        stepTime += hrclock.toMilliSeconds(t2 - t1);

        cout << "z: " << mockSensor.getLastSample().z
             << "\tvz: " << mockSensor.getLastSample().vz
             << "\tvMod: " << mockSensor.getLastSample().vMod
             << "\tservo: " << mockActuator.getCurrentPosition() << "\n";
    }

    miosix::Thread::sleep(AirBrakesConfigs::ABK_UPDATE_PERIOD);

    printf(
        "Init time: %f ms\nAvg iter time: %f ms\nTotal computation time: %f "
        "ms\n",
        startTime, stepTime / (LEN_TEST - 1), stepTime + startTime);

    // avoid reboot
    while (1)
    {
    }
}
