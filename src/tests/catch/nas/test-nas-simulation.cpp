/*
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Conterio
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

#ifdef STANDALONE_CATCH1_TEST
#include "catch/catch-tests-entry.cpp"
#endif

#define private public

#define EIGEN_NO_MALLOC  // enable eigen malloc usage assert

#include <NavigationAttitudeSystem/NASController.h>
#include <miosix.h>
#include <mocksensors/MockSensors.h>

#include <utils/testutils/catch.hpp>

#include "events/Events.h"
#include "utils/testutils/TestHelper.h"

using namespace DeathStackBoard;
using namespace NASConfigs;

MockIMU mock_imu;
MockPressureSensor mock_baro;
MockGPS mock_gps;

using NASCtrl = NASController<MockIMUData, PressureData, GPSData>;

NASCtrl controller(mock_imu, mock_baro, mock_gps);

void sampleSensors()
{
    mock_imu.sample();
    mock_baro.sample();
    mock_gps.sample();
}

void signalLiftoff()
{
    mock_baro.signalLiftoff();
    mock_imu.signalLiftoff();
    mock_gps.signalLiftoff();
}

TEST_CASE("Testing Navigation System Controller")
{
    TimestampTimer::enableTimestampTimer();

    sEventBroker->start();
    controller.start();

    EventCounter counter{*sEventBroker};
    counter.subscribe(TOPIC_NAS);

    // Startup: we should be in idle
    Thread::sleep(100);
    REQUIRE(controller.testState(&NASCtrl::state_idle));

    // Enter Calibrating and REQUIRE
    sEventBroker->post(Event{EV_CALIBRATE_NAS}, TOPIC_NAS);
    Thread::sleep(100);
    REQUIRE(controller.testState(&NASCtrl::state_calibrating));

    Thread::sleep(100);

    // sample some data for calibration
    for (uint32_t i = 0; i < CALIBRATION_N_SAMPLES; i++)
    {
        sampleSensors();
        Thread::sleep(10);
        controller.update();
    }

    // check calibrator values :
    // they should be equal to the last sample of each sensor
    // i.e. the first element of the arrays from which the sensors get the data
    PressureData press_data =
        mock_baro.getLastSample();  // still before liftoff
                                    // (same as SIMULATED_PRESSURE[0])
    GPSData gps_data = mock_gps.getLastSample();  // still before liftoff ...
    MockIMUData imu_data =
        mock_imu.getLastSample();  // still before liftoff ...
    NASReferenceValues ref_values{
        press_data.press, gps_data.latitude, gps_data.longitude,
        imu_data.accel_x, imu_data.accel_y,  imu_data.accel_z,
        imu_data.mag_x,   imu_data.mag_y,    imu_data.mag_z};
    REQUIRE(ref_values == controller.calibrator.getReferenceValues());

    // Now we should be in ready
    sampleSensors();
    controller.update();
    Thread::sleep(100);
    REQUIRE(controller.testState(&NASCtrl::state_ready));

    Thread::sleep(100);

    // retry the calibration phase
    sEventBroker->post({EV_CALIBRATE_NAS}, TOPIC_NAS);
    Thread::sleep(100);
    REQUIRE(controller.testState(&NASCtrl::state_calibrating));
    // sample some data for calibration
    for (uint32_t i = 0; i < CALIBRATION_N_SAMPLES + 10; i++)
    {
        sampleSensors();
        Thread::sleep(10);
        controller.update();
    }
    // check calibrator values :
    // they should be equal to the last sample of each sensor
    // i.e. the first element of the arrays from which the sensors get the data
    press_data = mock_baro.getLastSample();  // still before liftoff
                                             // (same as SIMULATED_PRESSURE[0])
    gps_data   = mock_gps.getLastSample();   // still before liftoff ...
    imu_data   = mock_imu.getLastSample();   // still before liftoff ...
    ref_values = {press_data.press, gps_data.latitude, gps_data.longitude,
                  imu_data.accel_x, imu_data.accel_y,  imu_data.accel_z,
                  imu_data.mag_x,   imu_data.mag_y,    imu_data.mag_z};
    REQUIRE(ref_values == controller.calibrator.getReferenceValues());

    // Now we should be in ready
    sampleSensors();
    controller.update();
    Thread::sleep(100);
    REQUIRE(controller.testState(&NASCtrl::state_ready));

    // Send liftoff event: should be in state active
    sEventBroker->post({EV_LIFTOFF}, TOPIC_FLIGHT_EVENTS);
    signalLiftoff();
    Thread::sleep(100);
    REQUIRE(controller.testState(&NASCtrl::state_active));

    // test the NAS on the mock sensors' data
    for (uint32_t i = 0; i < 2570; i++)
    {
        TRACE("Iteration: %u \n", i);
        sampleSensors();
        // Thread::sleep(10);
        controller.update();
    }
}
