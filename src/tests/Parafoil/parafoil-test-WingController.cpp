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
#include <Parafoil/AltitudeTrigger/AltitudeTrigger.h>
#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Buses.h>
#include <Parafoil/Configs/SensorsConfig.h>
#include <Parafoil/ParafoilModule/ParafoilModule.h>
#include <Parafoil/PinHandler/PinHandler.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Parafoil/StateMachines/NASController/NASController.h>
#include <Parafoil/StateMachines/WingController/WingController.h>
#include <Parafoil/TMRepository/TMRepository.h>
#include <Parafoil/WindEstimationScheme/WindEstimation.h>
#include <Parafoil/Wing/AutomaticWingAlgorithm.h>
#include <Parafoil/Wing/FileWingAlgorithm.h>
#include <common/Events.h>
#include <events/EventBroker.h>
#include <events/EventData.h>
#include <miosix.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace miosix;
using namespace Boardcore;
using namespace Parafoil;
using namespace Common;

int main()
{
    ModuleManager& modules = ModuleManager ::getInstance();
    PrintLogger logger     = Logging::getLogger("main");
    Logger::getInstance().start();

    bool initResult = true;

    // Initialize the modules
    {
        if (!modules.insert<Actuators>(new Actuators()))
        {
            initResult = false;
            LOG_ERR(logger, "Error initializing Actuators module");
        }

        if (!modules.insert<AltitudeTrigger>(new AltitudeTrigger()))
        {
            initResult = false;
            LOG_ERR(logger, "Error initializing AltitudeTrigger module");
        }

        if (!modules.insert<WindEstimation>(new WindEstimation()))
        {
            initResult = false;
            LOG_ERR(logger, "Error initializing WindEstimation module");
        }

        if (!modules.insert<WingController>(new WingController()))
        {
            initResult = false;
            LOG_ERR(logger, "Error initializing WingController module");
        }
    }

    // Start the modules
    {

        if (!Logger::getInstance().start())
        {
            initResult = false;
            LOG_ERR(logger, "Error initializing the Logger");
        }
        if (!modules.get<Actuators>()->startModule())
        {
            initResult = false;
            LOG_ERR(logger, "Error initializing Actuators module");
        }

        if (!modules.get<AltitudeTrigger>()->startModule())
        {
            initResult = false;
            LOG_ERR(logger, "Error initializing AltitudeTrigger module");
        }

        if (!modules.get<WindEstimation>()->startModule())
        {
            initResult = false;
            LOG_ERR(logger, "Error initializing WindEstimation module");
        }

        if (!modules.get<WingController>()->startModule())
        {
            initResult = false;
            LOG_ERR(logger, "Error initializing WingController module");
        }
    }
    if (initResult)
    {
        int i = 0;
        while (true)
        {
            if (i % 2 == 0)
            {
                modules.get<WingController>()->setAutomatic(false);
            }
            else
            {
                modules.get<WingController>()->setAutomatic(true);
            }
            i++;
            Thread::sleep(1000);
            // activate the WES
            EventBroker::getInstance().post(FLIGHT_WING_ALT_PASSED,
                                            TOPIC_ALGOS);
            Thread::sleep(1000);
            while (

                modules.get<WingController>()->getStatus().state ==
                WingControllerState::CALIBRATION)
            {  // wait until we change state

                TRACE("1servo Right: %f, %d Left: %f \n\n",
                      modules.get<Actuators>()->getServoPosition(
                          ServosList::PARAFOIL_RIGHT_SERVO),
                      modules.get<WingController>()->getStatus().state,
                      modules.get<Actuators>()->getServoPosition(
                          ServosList::PARAFOIL_LEFT_SERVO));
                Thread::sleep(1000);
            }
            while (modules.get<WingController>()->getStatus().state !=
                   WingControllerState::CALIBRATION)
            {  // wait until we return to WES
                TRACE("3servo Right: %f, %d Left: %f \n\n",
                      modules.get<Actuators>()->getServoPosition(
                          ServosList::PARAFOIL_RIGHT_SERVO),
                      modules.get<WingController>()->getStatus().state,
                      modules.get<Actuators>()->getServoPosition(
                          ServosList::PARAFOIL_LEFT_SERVO));
                Thread::sleep(1000);
            }
        }
    }
}
