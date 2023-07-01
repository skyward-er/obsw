/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Radu Raul
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

#include <Payload/BoardScheduler.h>
#include <Payload/Configs/FailSafeConfig.h>
#include <Payload/NASControllerMock/NASControllerMock.h>
#include <Payload/VerticalVelocityTrigger/VerticalVelocityTrigger.h>
#include <events/EventBroker.h>
#include <miosix.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace Boardcore;
using namespace Payload;

// Initial velocity for the Vertical Velocity Trigger
constexpr float MOCK_INITIAL_VERTICAL_VELOCITY = -30;
constexpr float MOCK_VELOCITY_DECELERATION     = 0.5;  // m/s^2

/**
 * This class mocks the NASController class since for this test we cannot start
 * the NASController as it needs the sensor reading and we cannot test the
 * velocity trigger unless we launch the rocket
 */
class NASMock : public NASController
{
public:
    /**
     * Default constructor for NASMock
     * It initializes the mockedVerticalSpeed
     */
    NASMock() : mockedVerticalSpeed(MOCK_INITIAL_VERTICAL_VELOCITY) {}

    /**
     * Default destructor for NASMock
     */
    ~NASMock() {}

    /**
     * This method mocks the NASController::getNasState() and returns a
     * NASState with a personalized vertical velocity that starts at
     * NASMock::MOCK_INITIAL_VERTICAL_VELOCITY and decreases by 0.5 m/s^2
     *
     * @return NASState object that contains the vertical vector in NED
     * orientation
     */
    NASState getNasState() override
    {
        mockedVerticalSpeed +=
            MOCK_VELOCITY_DECELERATION *
            ((float)FailSafe::FAILSAFE_VERTICAL_VELOCITY_TRIGGER_PERIOD /
             1000.f);
        NASState n = NASState();
        n.vd       = mockedVerticalSpeed;
        return n;
    }

private:
    // Keeps trace of the vertical velocity in NED orientation
    float mockedVerticalSpeed;
};

/**
 * This function halts the board
 */
void halt()
{
    while (1)
    {
    }
}

/**
 * This function calculates the maximum wait time needed by the Vertical
 * Velocity Trigger to trigger
 */
float calculateMaxWaitTime()
{
    float confidenceTime =
        ((float)FailSafe::FAILSAFE_VERTICAL_VELOCITY_TRIGGER_CONFIDENCE *
         (float)FailSafe::FAILSAFE_VERTICAL_VELOCITY_TRIGGER_PERIOD) /
        1000.f;
    float targetReachTime = (-MOCK_INITIAL_VERTICAL_VELOCITY -
                             FailSafe::FAILSAFE_VERTICAL_VELOCITY_THRESHOLD) /
                            MOCK_VELOCITY_DECELERATION;
    TRACE("Target reach time: %f [s]\n", targetReachTime);
    TRACE("Confidence time: %f [s]\n", confidenceTime);

    return confidenceTime + targetReachTime + 1;
}

int main()
{

    // Insert the modules
    ModuleManager::getInstance().insert<NASController>(new NASMock());
    ModuleManager::getInstance().insert<VerticalVelocityTrigger>(
        new VerticalVelocityTrigger());

    // start the scheduler
    TRACE("Starting the board scheduler\n");
    if (!BoardScheduler::getInstance().getScheduler().start())
    {
        TRACE("Error starting the General Purpose Scheduler\n");
        halt();
    }

    // start the modules
    if (!ModuleManager::getInstance().get<NASController>()->start())
    {
        TRACE("Error starting the NAS Controller\n");
        halt();
    }
    if (!ModuleManager::getInstance()
             .get<VerticalVelocityTrigger>()
             ->startModule())
    {
        TRACE("Error starting the Vertical Velocity Trigger\n");
        halt();
    }

    ModuleManager::getInstance().get<VerticalVelocityTrigger>()->enable();

    TRACE("Starting... \n");
    // wait for the trigger
    int count       = 0;
    int maxWaitTime = calculateMaxWaitTime() * 100;
    while (
        ModuleManager::getInstance().get<VerticalVelocityTrigger>()->isActive())
    {
        Thread::sleep(100);
        count++;

        if (count > maxWaitTime)
        {
            TRACE("Vertical Velocity Trigger not working properly\n");
            break;
        }
    }
    if (count < maxWaitTime)
    {
        TRACE("Vertical Velocity Triggered \n");
    }

    halt();
}
