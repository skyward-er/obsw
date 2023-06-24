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

constexpr float MOCK_INITIAL_VERTICAL_VELOCITY = -15;

class NASMock : public NASController
{
public:
    // default constructor
    NASMock() : mockedVerticalSpeed(MOCK_INITIAL_VERTICAL_VELOCITY) {}

    // default destructor
    ~NASMock() {}

    // mocked version of the getNasState function
    // it returns a NASState with a mocked vertical velocity
    // that is increased by initalVerticalVelocity [s] / 9.81 [m/s^2] every time
    // it is called. We suppose that the gravity is applied directly to the
    // vertical component
    NASState getNasState()
    {
        mockedVerticalSpeed +=
            (FailSafe::FAILSAFE_VERTICAL_VELOCITY_TRIGGER_PERIOD / 1000) * 9.81;
        TRACE("[*] mockedVerticalSpeed: %f\n", mockedVerticalSpeed);
        NASState n = NASState();
        n.vd       = mockedVerticalSpeed;
        return n;
    }

private:
    float mockedVerticalSpeed;
};

void halt()
{
    while (1)
    {
    }
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

    float vel =
        ModuleManager::getInstance().get<NASController>()->getNasState().vd;
    TRACE("Testing if the mock is accessible: Velocity: %f\n", vel);
    if (vel == 0)
    {
        TRACE("Test failed: could not access the mock class!\n");
        halt();
    }

    ModuleManager::getInstance().get<VerticalVelocityTrigger>()->enable();

    TRACE("Starting... \n");
    // wait for the trigger
    int count       = 0;
    int maxWaitTime = 3 * ((-MOCK_INITIAL_VERTICAL_VELOCITY / 9.81 * 100) + 1);
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
