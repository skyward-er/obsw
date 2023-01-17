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

#include <Parafoil/Actuators/Actuators.h>
#include <Parafoil/AltitudeTrigger/AltitudeTrigger.h>
#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Configs/SensorsConfig.h>
#include <Parafoil/PinHandler/PinHandler.h>
#include <Parafoil/Radio/Radio.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Parafoil/StateMachines/NASController/NASController.h>
#include <Parafoil/StateMachines/WingController/WingController.h>
#include <Parafoil/Wing/AutomaticWingAlgorithm.h>
#include <Parafoil/Wing/FileWingAlgorithm.h>
#include <common/events/Events.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/PrintLogger.h>
#include <events/EventBroker.h>
#include <events/EventData.h>
#include <events/utils/EventSniffer.h>
#include <miosix.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#ifdef HILSimulation
#include <HIL.h>
#include <HIL_algorithms/HILMockAerobrakeAlgorithm.h>
#include <HIL_algorithms/HILMockKalman.h>
#include <HIL_sensors/HILSensors.h>
#endif

using namespace miosix;
using namespace Boardcore;
using namespace Parafoil;
using namespace Common;

int main()
{
    bool initResult        = true;
    ModuleManager& modules = ModuleManager::getInstance();
    PrintLogger logger     = Logging::getLogger("main");

    // Initialize the modules
    modules.insert<Actuators>(new Actuators());
    modules.insert<AltitudeTrigger>(new AltitudeTrigger());
    modules.insert<BoardScheduler>(new BoardScheduler());
    modules.insert<PinHandler>(new PinHandler());
    modules.insert<Radio>(new Radio());
    modules.insert<Sensors>(new Sensors());

#ifdef HILSimulation
    auto flightPhasesManager = HIL::getInstance().flightPhasesManager;

    flightPhasesManager->setCurrentPositionSource(
        []() {
            return TimedTrajectoryPoint{
                NASController::getInstance().getNasState()};
        });

    HIL::getInstance().start();

    modules.get<BoardScheduler>()->getScheduler().addTask(
        []() { HIL::getInstance().send(0.0f); }, 100);

    // flightPhasesManager->registerToFlightPhase(FlightPhases::FLYING, )
#endif

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

    // Initialize the servo outputs
    if (!modules.get<Actuators>()->enableServo(PARAFOIL_LEFT_SERVO) ||
        !modules.get<Actuators>()->setServo(PARAFOIL_LEFT_SERVO, 0) ||
        !modules.get<Actuators>()->enableServo(PARAFOIL_RIGHT_SERVO) ||
        !modules.get<Actuators>()->setServo(PARAFOIL_RIGHT_SERVO, 0))
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Actuators");
    }

    // Start the radio
    if (!modules.get<Radio>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the radio");
    }

    // Start the state machines
    if (!FlightModeManager::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the FlightModeManager");
    }

    if (!NASController::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the NAS algorithm");
    }

    if (!WingController::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the WingController");
    }
    WingController::getInstance().setControlled(false);  // set the algorithm to
    // sequence
    //   Start the sensors sampling
    if (!modules.get<Sensors>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the sensors");
    }

    // Start the board task scheduler
    if (!modules.get<BoardScheduler>()->getScheduler().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the General Purpose Scheduler");
    }

    // Start the pin handler and observer
    if (!PinObserver::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the PinObserver");
    }

    // If all is correctly set up i publish the init ok
    if (initResult)
        EventBroker::getInstance().post(FMM_INIT_OK, TOPIC_FMM);
    else
        EventBroker::getInstance().post(FMM_INIT_ERROR, TOPIC_FMM);

    // Log all events
    EventSniffer sniffer(
        EventBroker::getInstance(), TOPICS_LIST,
        [](uint8_t event, uint8_t topic)
        {
            EventData ev{TimestampTimer::getTimestamp(), event, topic};
            Logger::getInstance().log(ev);
        });

    // Periodically statistics
    while (true)
    {
        Thread::sleep(1000);
        Logger::getInstance().log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();
        Logger::getInstance().logStats();
        modules.get<Radio>()->logStatus();
        StackLogger::getInstance().log();
    }
}
