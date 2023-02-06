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
    NASMock() : NASController(), mocked_altitude(0) {}

    // default destructor
    ~NASMock() {}

    NASState getNasState() override
    {
        mocked_altitude += 2.5;
        NASState n = NASState();
        n.d        = mocked_altitude;
        return n;
    }

private:
    float mocked_altitude;
};

int main()
{
    ModuleHelper& module_helper = ModuleHelper::getInstance();
    PrintLogger logger          = Logging::getLogger("main");
    NASMock nas                 = NASMock();  // FIXME
    // EventBroker& broker         = EventBroker::getInstance();

    // Mock up the modules
    if (!module_helper.mockUp<NASController>(&nas, ModuleType::NASController))
    {
        LOG_ERR(logger, "Error mocking up the NAS Controller");
    }

    // Initialize the modules
    module_helper.setUpAltitudeTrigger();

    // start the modules
    module_helper.startAllModules();

    // get the modules
    ModuleManager& modules = module_helper.getModules();
    AltitudeTrigger* alt   = modules.get<AltitudeTrigger>();

    // start the scheduler
    if (!modules.get<BoardScheduler>()->getScheduler().start())
    {
        LOG_ERR(logger, "Error starting the General Purpose Scheduler");
    }

    // enable the altitude trigger
    alt->enable();

    // wait for the trigger
    int count = 0;
    while (alt->isActive())
    {
        Thread::sleep(100);
        count++;

        if (count > 100)
        {
            LOG_ERR(logger, "Altitude Trigger not working");
            break;
        }
    }

    LOG_INFO(logger, "Altitude Triggered");
}