/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <Main/Actuators/Actuators.h>
#include <Main/BoardScheduler.h>
#include <Main/CanHandler/CanHandler.h>
#include <Main/PinHandler/PinHandler.h>
#include <Main/Radio/Radio.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/ADAController/ADAController.h>
#include <Main/StateMachines/AirBrakesController/AirBrakesController.h>
#include <Main/StateMachines/Deployment/Deployment.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Main/StateMachines/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Main/StateMachines/NASController/NASController.h>
#include <common/events/Events.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <events/EventBroker.h>
#include <miosix.h>
#include <utils/PinObserver/PinObserver.h>

#include "kernel/scheduler/priority/priority_scheduler.h"

using namespace miosix;
using namespace Boardcore;
using namespace Main;
using namespace Common;

int main()
{
    bool initResult    = true;
    PrintLogger logger = Logging::getLogger("main");

    if (!Logger::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting SD logger");
    }

    if (!EventBroker::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the EventBroker");
    }

    // Start the radio
    if (!Radio::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the radio");
    }

    // Start the can interface
    if (!CanHandler::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the CAN interface");
    }

    // Start the FMM
    if (!FlightModeManager::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the FMM");
    }

    // Start the state machines
    if (!ADAController::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the ADAController");
    }
    if (!AirBrakesController::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the AirBrakesController");
    }
    if (!Deployment::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Deployment");
    }
    if (!FlightModeManager::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the FlightModeManager");
    }
    if (!FlightStatsRecorder::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the FlightStatsRecorder");
    }
    if (!NASController::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the NASController");
    }

    // Start the sensors sampling
    if (!Sensors::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the sensors");
    }

    // Start the pin handler and observer
    if (!PinObserver::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the PinObserver");
    }

    // Start the board task scheduler
    if (!BoardScheduler::getInstance().getScheduler().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the BoardScheduler");
    }

    // If all is correctly set up i publish the init ok
    if (initResult)
        EventBroker::getInstance().post(FMM_INIT_OK, TOPIC_FMM);
    else
        EventBroker::getInstance().post(FMM_INIT_ERROR, TOPIC_FMM);

    // Periodical statistics
    while (true)
    {
        Thread::sleep(1000);
        Logger::getInstance().log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();
        Logger::getInstance().logStats();
        Radio::getInstance().logStatus();
    }
}
