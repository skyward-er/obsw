/* Copyright (c) 2019-2021 Skyward Experimental Rocketry
 * Authors: Luca Mozzarelli, Luca Conterio
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

#ifdef STANDALONE_CATCH1_TEST
#include "catch/catch-tests-entry.cpp"
#endif

#define EIGEN_NO_MALLOC

#include <Common.h>
#include <events/EventBroker.h>
#include <events/Events.h>
#include <events/FSM.h>
#include <events/utils/EventCounter.h>

#define private public
#define protected public

#include <ApogeeDetectionAlgorithm/ADAAlgorithm.h>
#include <ApogeeDetectionAlgorithm/ADAController.h>
#include <DeathStack.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <utils/testutils/catch.hpp>

#include "test-ada-data.h"

using namespace DeathStackBoard;
using namespace ADAConfigs;

constexpr float NOISE_STD_DEV                = 5;  // Noise variance
constexpr float LSB                          = 28;
constexpr unsigned int SHADOW_MODE_END_INDEX = 30;
constexpr unsigned int APOGEE_SAMPLE         = 382;

// Mock sensors for testing purposes
class MockPressureSensor : public Sensor<PressureData>
{
public:
    MockPressureSensor() {}

    bool init() override { return true; }

    bool selfTest() override { return true; }

    PressureData sampleImpl() override
    {
        float press = 0.0;

        if (before_liftoff)
        {
            press = addNoise(ADA_SIMULATED_PRESSURE[0]);
        }
        else
        {
            if (i < DATA_SIZE)
            {
                press = addNoise(ADA_SIMULATED_PRESSURE[i++]);
            }
            else
            {
                press = addNoise(ADA_SIMULATED_PRESSURE[DATA_SIZE - 1]);
            }
        }

        return PressureData{TimestampTimer::getTimestamp(), press};
    }

    void signalLiftoff() { before_liftoff = false; }

private:
    volatile bool before_liftoff = true;
    volatile unsigned int i      = 0;  // Last index
    std::default_random_engine generator{1234567};
    std::normal_distribution<float> distribution{0.0f, NOISE_STD_DEV};

    float addNoise(float sample)
    {
        return quantization(sample + distribution(generator));
    }

    float quantization(float sample) { return round(sample / LSB) * LSB; }
};

class MockGPSSensor : public Sensor<GPSData>
{
public:
    bool init() { return true; }
    bool selfTest() { return true; }
    GPSData sampleImpl() { return GPSData{}; }
};

MockPressureSensor mock_baro;
MockGPSSensor mock_gps;

using ADACtrl = ADAController<PressureData, GPSData>;
ADACtrl *ada_controller;

void checkState(unsigned int i, ADAKalmanState state)
{
    if (i > 200)
    {
        if (state.x0 == Approx(ADA_SIMULATED_PRESSURE[i]).margin(70))
            SUCCEED();
        else
            FAIL("i = " << i << "\t\t" << state.x0
                        << " != " << ADA_SIMULATED_PRESSURE[i]);

        // Flying under the chutes the speed estimation is not very precise
        if (i < 3000)
        {
            if (state.x1 == Approx(ADA_SIMULATED_PRESSURE_SPEED[i]).margin(80))
                SUCCEED();
            else
                FAIL("i = " << i << "\t\t" << state.x1
                            << " != " << ADA_SIMULATED_PRESSURE_SPEED[i]);
        }
    }
}

TEST_CASE("Testing ada_controller from calibration to first descent phase")
{
    TimestampTimer::enableTimestampTimer();

    ada_controller = new ADACtrl(mock_baro, mock_gps);
    TRACE("ADA init : %d \n", ada_controller->start());

    // Start event broker and ada_controller
    sEventBroker->start();
    EventCounter counter{*sEventBroker};
    counter.subscribe(TOPIC_ADA);

    // Startup: we should be in idle
    Thread::sleep(100);
    REQUIRE(ada_controller->testState(&ADACtrl::state_idle));

    // Enter Calibrating and REQUIRE
    sEventBroker->post({EV_CALIBRATE_ADA}, TOPIC_ADA);
    Thread::sleep(100);
    REQUIRE(ada_controller->testState(&ADACtrl::state_calibrating));

    // Send baro calibration samples
    for (unsigned i = 0; i < CALIBRATION_BARO_N_SAMPLES + 5; i++)
    {
        mock_baro.sample();
        Thread::sleep(10);
        ada_controller->update();
    }

    float mean = ada_controller->calibrator.getReferenceValues().ref_pressure;
    if (mean == Approx(ADA_SIMULATED_PRESSURE[0]))
        FAIL("Calibration value");
    else
        SUCCEED();

    // Should still be in calibrating
    Thread::sleep(100);
    REQUIRE(ada_controller->testState(&ADACtrl::state_calibrating));

    // Send set deployment altitude
    ada_controller->setDeploymentAltitude(100);
    mock_baro.sample();
    Thread::sleep(10);
    ada_controller->update();

    // Should still be in calibrating
    Thread::sleep(100);
    REQUIRE(ada_controller->testState(&ADACtrl::state_calibrating));

    // Send set altitude ref
    ada_controller->setReferenceAltitude(1300);
    mock_baro.sample();
    Thread::sleep(10);
    ada_controller->update();
    // Should still be in calibrating
    Thread::sleep(100);
    REQUIRE(ada_controller->testState(&ADACtrl::state_calibrating));

    // Send set temperature ref
    ada_controller->setReferenceTemperature(15);
    mock_baro.sample();
    Thread::sleep(10);
    ada_controller->update();

    // Now we should be in ready
    Thread::sleep(100);
    mock_baro.sample();
    Thread::sleep(10);
    ada_controller->update();
    Thread::sleep(100);
    REQUIRE(ada_controller->testState(&ADACtrl::state_ready));

    // Send liftoff event: should be in shadow mode
    sEventBroker->post({EV_LIFTOFF}, TOPIC_FLIGHT_EVENTS);
    mock_baro.signalLiftoff();
    Thread::sleep(100);
    REQUIRE(ada_controller->testState(&ADACtrl::state_shadowMode));
    long long shadow_mode_start = miosix::getTick();

    // Perform some chcmathecks while in shadow mode (to avoid triggering a
    // false apogee)
    for (unsigned i = 0; i < SHADOW_MODE_END_INDEX; i++)
    {
        // float noisy_p = addNoise(ADA_SIMULATED_PRESSURE[i]);
        mock_baro.sample();
        Thread::sleep(5);
        ada_controller->update();
        float noisy_p = mock_baro.getLastSample().press;
        // Thread::sleep(100);
        ADAKalmanState state = ada_controller->ada.getKalmanState();
        printf("%d,%f,%f,%f\n", (int)i, noisy_p, state.x0,
               ada_controller->ada.getVerticalSpeed());
        checkState(i, state);
    }

    // Wait timeout
    Thread::sleepUntil(shadow_mode_start + TIMEOUT_ADA_SHADOW_MODE);
    // Should be active now
    REQUIRE(ada_controller->testState(&ADACtrl::state_active));

    Thread::sleep(100);
    // Send samples
    bool apogee_checked = false;
    for (unsigned i = SHADOW_MODE_END_INDEX; i < DATA_SIZE; i++)
    {
        // float noisy_p = addNoise(ADA_SIMULATED_PRESSURE[i]);
        mock_baro.sample();
        Thread::sleep(5);
        ada_controller->update();
        float noisy_p = mock_baro.getLastSample().press;
        // Thread::sleep(100);
        ADAKalmanState state = ada_controller->ada.getKalmanState();
        printf("%d,%f,%f,%f\n", (int)i, noisy_p, state.x0,
               ada_controller->ada.getVerticalSpeed());
        checkState(i, state);

        if (ada_controller->getStatus().apogee_reached == true &&
            !apogee_checked)
        {
            apogee_checked = true;
            if (fabs(i - APOGEE_SAMPLE) > 10)
            {
                FAIL("Apogee error: " << (int)i - APOGEE_SAMPLE << " samples");
            }
            else
            {
                printf("Apogee error: %d samples\n", (int)(i - APOGEE_SAMPLE));
                SUCCEED();
            }

            Thread::sleep(1000);
            REQUIRE(ada_controller->testState(
                &ADACtrl::state_pressureStabilization));
            Thread::sleep(EV_TIMEOUT_PRESS_STABILIZATION + 1000);
            REQUIRE(ada_controller->testState(&ADACtrl::state_drogueDescent));
        }
    }
}