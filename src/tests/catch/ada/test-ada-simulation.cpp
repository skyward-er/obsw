/*
 * Copyright (c) 2019 Skyward Experimental Rocketry
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

#define private public
#define protected public

#ifdef STANDALONE_CATCH1_TEST
#include "catch/catch-tests-entry.cpp"
#endif

#include <Common.h>
#include <DeathStack/ADA/ADA.h>
#include <DeathStack/events/Events.h>
#include <events/EventBroker.h>
#include <events/FSM.h>
#include <events/utils/EventCounter.h>
#include <algorithm>
#include <iostream>
#include <random>
#include <sstream>
#include <utils/testutils/catch.hpp>
#include "test-ada-data.h"

using namespace DeathStackBoard;

constexpr float NOISE_STD_DEV = 5;  // Noise varaince
constexpr float LSB           = 28;

ADA *ada;
unsigned seed = 1234567;  // Seed for noise generation

float addNoise(float sample);  // Function to add noise
float quantization(float sample);
std::default_random_engine generator(seed);  // Noise generator
std::normal_distribution<float> distribution(
    0.0, NOISE_STD_DEV);  // Noise generator distribution

typedef miosix::Gpio<GPIOG_BASE, 13> greenLed;

TEST_CASE("Testing ADA from calibration to first descent phase")
{
    // Setting pin mode for signaling ADA status
    {
        miosix::FastInterruptDisableLock dLock;
        greenLed::mode(miosix::Mode::OUTPUT);
    }

    ada = new ADA();

    // Start event broker and ada
    sEventBroker->start();
    ada->start();
    EventCounter counter{*sEventBroker};
    counter.subscribe(TOPIC_ADA);

    // Startup: we should be in idle
    Thread::sleep(100);
    CHECK(ada->testState(&ADA::stateIdle));

    // Enter Calibrating and check
    sEventBroker->post({EV_CALIBRATE_ADA}, TOPIC_TC);
    Thread::sleep(100);
    CHECK(ada->testState(&ADA::stateCalibrating));

    // Send baro calibration samples
    for (unsigned i = 0; i < CALIBRATION_BARO_N_SAMPLES + 5; i++)
    {
        ada->updateBaro(addNoise(SIMULATED_PRESSURE[0]));
    }

    float mean = ada->pressure_stats.getStats().mean;
    if (mean == Approx(SIMULATED_PRESSURE[0]))
        FAIL("Calibration value");
    else
        SUCCEED();

    // Should still be in calibrating
    Thread::sleep(100);
    CHECK(ada->testState(&ADA::stateCalibrating));

    // Send set deployment altitude
    ada->setDeploymentAltitude(100);
    ada->updateBaro(addNoise(SIMULATED_PRESSURE[0]));

    // Should still be in calibrating
    Thread::sleep(100);
    CHECK(ada->testState(&ADA::stateCalibrating));

    // Send set altitude ref
    ada->setReferenceAltitude(1300);
    ada->updateBaro(addNoise(SIMULATED_PRESSURE[0]));

    // Should still be in calibrating
    Thread::sleep(100);
    CHECK(ada->testState(&ADA::stateCalibrating));

    // Send set temperature ref
    ada->setReferenceTemperature(15);
    ada->updateBaro(addNoise(SIMULATED_PRESSURE[0]));

    // Now we should be in ready
    Thread::sleep(100);
    ada->updateBaro(addNoise(SIMULATED_PRESSURE[0]));
    Thread::sleep(100);
    CHECK(ada->testState(&ADA::stateReady));

    sEventBroker->post({EV_LIFTOFF}, TOPIC_FLIGHT_EVENTS);

    // Send liftoff event: should be in shadow mode
    Thread::sleep(100);
    CHECK(ada->testState(&ADA::stateShadowMode));

    // Wait timeout
    Thread::sleep(TIMEOUT_ADA_SHADOW_MODE);
    // Should be active now
    CHECK(ada->testState(&ADA::stateActive));

    Thread::sleep(100);
    // Send samples

    printf("%d\n", DATA_SIZE);

    for (unsigned i = 0; i < DATA_SIZE / 2; i++)
    {
        greenLed::high();
        ada->updateBaro(addNoise(SIMULATED_PRESSURE[i]));
        // Thread::sleep(100);
        KalmanState state = ada->getKalmanState();

        if (i > 200)
        {
            if (state.x0 == Approx(SIMULATED_PRESSURE[i]).margin(70))
                SUCCEED();
            else
                FAIL("i = " << i << "\t\t" << state.x0
                            << " != " << SIMULATED_PRESSURE[i]);

            if (state.x1 == Approx(SIMULATED_PRESSURE_SPEED[i]).margin(80))
                SUCCEED();
            else
                FAIL("i = " << i << "\t\t" << state.x1
                            << " != " << SIMULATED_PRESSURE_SPEED[i]);
        }

        if (ada->getStatus().apogee_reached == true)
        {
            if (i != Approx(367.0f).margin(20))
                FAIL("Apogee error: " << (int)i - 367 << " samples");
            else
                SUCCEED();
        }
        greenLed::low();
    }
}

float addNoise(float sample)
{
    float noise = distribution(generator);
    return quantization(sample + noise);
}

float quantization(float sample) { return round(sample / LSB) * LSB; }