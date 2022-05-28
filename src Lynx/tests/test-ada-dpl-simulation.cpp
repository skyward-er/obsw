/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

#include <ApogeeDetectionAlgorithm/ADAAlgorithm.h>
#include <ApogeeDetectionAlgorithm/ADAController.h>
#include <Deployment/DeploymentController.h>
#include <utils/Debug.h>

#include <random>

#include "PinHandler/PinHandler.h"
#include "catch/ada/ada_kalman_p/test-ada-data.h"
#include "events/EventBroker.h"
#include "events/Events.h"
#include "events/utils/EventCounter.h"

using namespace Boardcore;
using namespace DeathStackBoard;

constexpr float NOISE_STD_DEV = 5;  // Noise varaince
constexpr float LSB           = 28;

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

int main()
{
    MockPressureSensor baro;
    MockGPSSensor gps;

    if (!baro.init())
    {
        TRACE("Barometer init failed ... \n");
        for (;;)
            ;
    }

    ADAController<PressureData, GPSData> ada_controller(baro, gps);
    DeploymentController dpl_controller;
    PinHandler pin_handler;

    sEventBroker.start();
    EventCounter counter{sEventBroker};
    counter.subscribe(TOPIC_ADA);
    counter.subscribe(TOPIC_FLIGHT_EVENTS);

    Thread::sleep(1000);

    if (!pin_handler.start())
    {
        TRACE("[PinHnalder] Failed to start \n");
    }

    ada_controller.start();
    dpl_controller.start();

    Thread::sleep(1000);

    // Enter ADA calibration state
    sEventBroker.post({EV_CALIBRATE_ADA}, TOPIC_ADA);
    Thread::sleep(100);

    // Send baro calibration samples for ADA
    for (unsigned i = 0; i < ADAConfigs::CALIBRATION_BARO_N_SAMPLES + 5; i++)
    {
        baro.sample();
        Thread::sleep(50);
        ada_controller.update();
    }

    // Send set deployment altitude
    ada_controller.setDeploymentAltitude(200);
    baro.sample();
    Thread::sleep(50);
    ada_controller.update();
    // Send set altitude ref
    ada_controller.setReferenceAltitude(1300);
    baro.sample();
    Thread::sleep(50);
    ada_controller.update();
    // Send set temperature ref
    ada_controller.setReferenceTemperature(15);
    baro.sample();
    Thread::sleep(50);
    ada_controller.update();

    // wait for launch pin detach
    while (counter.getCount({EV_UMBILICAL_DETACHED}) == 0)
    {
        Thread::sleep(100);
    }

    TRACE("Sending EV_LIFTOFF ... \n");
    // Send liftoff event
    sEventBroker.post({EV_LIFTOFF}, TOPIC_FLIGHT_EVENTS);
    baro.signalLiftoff();

    bool first_apogee_detection       = true;
    bool first_dpl_altitude_detection = true;

    for (unsigned int i = 0; i < DATA_SIZE; i++)
    {
        baro.sample();
        Thread::sleep(50);
        ada_controller.update();

        TRACE("%u, %.2f, %.2f \n", i, ada_controller.getADAData().vert_speed,
              ada_controller.getADAData().msl_altitude);

        if (ada_controller.getStatus().apogee_reached && first_apogee_detection)
        {
            TRACE("\n\n*******   APOGEE DETECTED!   ******* \n\n");
            first_apogee_detection = false;

            TRACE("Triggering nosecone ejection ... \n\n");
            sEventBroker.post({EV_NC_OPEN}, TOPIC_DPL);
        }

        if (ada_controller.getStatus().dpl_altitude_reached &&
            first_dpl_altitude_detection)
        {
            TRACE("\n\n*******   DPL ALTITUDE DETECTED!   ******* \n\n");
            first_dpl_altitude_detection = false;

            TRACE("Triggering cutting sequence ... \n\n");
            sEventBroker.post({EV_CUT_DROGUE}, TOPIC_DPL);
        }
    }

    TRACE("\nLanded ... \n\n");

    for (;;)
    {
        Thread::sleep(1000);
    }

    return 0;
}
