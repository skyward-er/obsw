#include <miosix.h>

#include <cmath>
#include <cstdio>

#define private public

#include "boards/DeathStack/AeroBrakesController/AeroBrakesControllerAlgorithm.h"
#include "boards/DeathStack/AeroBrakesController/AerobrakesServoInterface.h"
#include "drivers/HardwareTimer.h"
#include "sensors/Sensor.h"
#include "testdata.h"

using namespace DeathStackBoard;

class MockActuator : public AerobrakesServoInterface
{
public:
    MockActuator() {}
    void sendData(float alpha) { lastAlpha = round(alpha * 180 / PI); }
    int error(int alphaTest) { return abs(alphaTest - lastAlpha); }

private:
    int lastAlpha;
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

MockActuator* ma           = new MockActuator;
MockSensor<InputClass>* ms = new MockSensor<InputClass>;
AeroBrakesControllerAlgorithm<InputClass> ca =
    AeroBrakesControllerAlgorithm<InputClass>(ms, ma);

int main()
{
    {
        miosix::FastInterruptDisableLock dLock;

        // Enable TIM5 peripheral clocks
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    }

    input_t input;

    input = DATA[0].input;

    HardwareTimer<uint32_t> hrclock(
        TIM5,
        TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1));
    hrclock.setPrescaler(127);
    hrclock.start();

    ms->sample();

    uint32_t t1 = hrclock.tick();
    ca.begin();
    uint32_t t2 = hrclock.tick();

    float startTime   = hrclock.toMilliSeconds(t2 - t1);
    float stepTime    = 0;
    int maxAlphaError = 0;

    for (int i = 1; i < LEN_TEST; i++)
    {
        ms->sample();
        t1 = hrclock.tick();
        ca.update();
        t2 = hrclock.tick();

        int error = ma->error(DATA[i].output.alpha);

        if (error > maxAlphaError)
        {
            maxAlphaError = error;
        }

        stepTime += hrclock.toMilliSeconds(t2 - t1);
    }

    printf("Max alpha error: %d\n", maxAlphaError);

    printf(
        "Init time: %f ms\nAvg iter time: %f ms\nTotal computation time: %f "
        "ms\n",
        startTime, stepTime / (LEN_TEST - 1), stepTime + startTime);
}