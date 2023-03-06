/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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

#include <Parafoil/Actuators/Actuators.h>
#include <Parafoil/BoardScheduler.h>
#include <Parafoil/StateMachines/WingController/WingController.h>
#include <common/Events.h>
#include <events/EventBroker.h>
#include <events/EventData.h>
#include <miosix.h>

#include <Parafoil/ModuleHelper/ModuleHelper.hpp>
#include <utils/ModuleManager/ModuleManager.hpp>

using namespace miosix;
using namespace Boardcore;
using namespace Parafoil;
using namespace Common;

int main()
{
    ModuleHelper& module_helper = ModuleHelper::getInstance();

    // Initialize the modules
    module_helper.setUpActuators();
    module_helper.setUpWindEstimation();
    module_helper.setUpWingController();
    module_helper.setUpNASController();

    module_helper.startAllModules();

    ModuleManager& modules = module_helper.getModules();
    PrintLogger logger     = Logging::getLogger("main");

    if (!EventBroker::getInstance().start())
    {
        LOG_ERR(logger, "Error starting the EventBroker");
    }

    // Initialize the servo outputs
    if (!modules.get<Actuators>()->enableServo(PARAFOIL_LEFT_SERVO) ||
        !modules.get<Actuators>()->setServo(PARAFOIL_LEFT_SERVO, 0) ||
        !modules.get<Actuators>()->enableServo(PARAFOIL_RIGHT_SERVO) ||
        !modules.get<Actuators>()->setServo(PARAFOIL_RIGHT_SERVO, 0))
    {
        LOG_ERR(logger, "Error starting the Actuators");
    }

    /*if (!modules.get<NASController>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the NAS algorithm");
    }*/

    if (!modules.get<WingController>()->start())
    {
        LOG_ERR(logger, "Error starting the WingController");
    }

    // Start the board task scheduler
    if (!BoardScheduler::getInstance().getScheduler().start())
    {
        LOG_ERR(logger, "Error starting the General Purpose Scheduler");
    }

    // activate the WES
    EventBroker::getInstance().post(WING_WES, TOPIC_ALGOS);
    int i = 0;
    while (true)
    {
        if (i % 2 == 0)
        {
            modules.get<WingController>()->setControlled(1);
        }
        else
        {
            modules.get<WingController>()->setControlled(1);
        }
        i++;
        while (modules.get<WingController>()->getStatus() ==
               WingControllerState::WES)
        {  // wait until we change state

            TRACE("1servo Right: %f, Left: %f \n\n",
                  modules.get<Actuators>()->getServoPosition(
                      ServosList::PARAFOIL_RIGHT_SERVO),
                  modules.get<Actuators>()->getServoPosition(
                      ServosList::PARAFOIL_LEFT_SERVO));
            Thread::sleep(1000);
        }
        while (modules.get<WingController>()->getStatus() !=
               WingControllerState::WES)
        {  // wait until we return to WES
            TRACE("3servo Right: %f, Left: %f \n\n",
                  modules.get<Actuators>()->getServoPosition(
                      ServosList::PARAFOIL_RIGHT_SERVO),
                  modules.get<Actuators>()->getServoPosition(
                      ServosList::PARAFOIL_LEFT_SERVO));
            Thread::sleep(500);
        }
    }
}
