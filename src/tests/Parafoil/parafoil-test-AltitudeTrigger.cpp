/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Federico Lolli
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

#include <Parafoil/AltitudeTrigger/AltitudeTrigger.h>
#include <Parafoil/BoardScheduler.h>
#include <events/EventBroker.h>
#include <miosix.h>

#include <Parafoil/ModuleHelper/ModuleHelper.hpp>
#include <utils/ModuleManager/ModuleManager.hpp>

// using namespace miosix;
using namespace Boardcore;
using namespace Parafoil;
// using namespace Common;

class NASMock : public NASController
{
public:
    // default constructor
    NASMock() : mocked_altitude(-500) {}

    // default destructor
    ~NASMock() {}

    NASState getNasState() override
    {
        mocked_altitude += 0.5;
        NASState n = NASState();
        n.d        = mocked_altitude;
        return n;
    }

private:
    float mocked_altitude;
};

int main()
{

    // ModuleHelper& module_helper = ModuleHelper::getInstance();
    // get the modules
    // PrintLogger logger          = Logging::getLogger("main");
    //  EventBroker& broker         = EventBroker::getInstance();
    // start the modules
    ModuleManager::getInstance().insert<BoardScheduler>(new BoardScheduler());
    // Initialize the modules
    // start the scheduler
    ModuleManager::getInstance().insert<NASController>(new NASMock());
    ModuleManager::getInstance().insert<AltitudeTrigger>(new AltitudeTrigger());
    if (!ModuleManager::getInstance().get<BoardScheduler>()->start())
    {
        TRACE("Error starting the General Purpose Scheduler\n");
    }

    if (!ModuleManager::getInstance().get<NASController>()->start())
    {
        TRACE("Error starting the NAS Controller\n");
    }
    if (!ModuleManager::getInstance().get<AltitudeTrigger>()->start())
    {
        TRACE("Error starting the NAS Controller\n");
    }

    ModuleManager::getInstance().get<AltitudeTrigger>()->enable();

    TRACE("Starting... \n");
    // wait for the trigger
    int count = 0;
    while (ModuleManager::getInstance().get<AltitudeTrigger>()->isActive())
    {
        Thread::sleep(100);
        count++;

        if (count > 100)
        {
            TRACE("Altitude Trigger not working \n");
            break;
        }
    }
    if (count < 100)
    {
        TRACE("Altitude Triggered \n");
    }
}