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
#include <Parafoil/Buses.h>
#include <Parafoil/Configs/SensorsConfig.h>
#include <Parafoil/PinHandler/PinHandler.h>
#include <Parafoil/Radio/Radio.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Parafoil/StateMachines/NASController/NASController.h>
#include <Parafoil/StateMachines/WingController/WingController.h>
#include <Parafoil/WindEstimationScheme/WindEstimation.h>
#include <Parafoil/Wing/AutomaticWingAlgorithm.h>
#include <Parafoil/Wing/FileWingAlgorithm.h>
#include <common/Events.h>
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

    if (!modules.insert<Buses>(new Buses()))
    {
        initResult = false;
        LOG_ERR(logger, "Error initializing Buses module");
    }

    if (!modules.insert<NASController>(new NASController()))
    {
        initResult = false;
        LOG_ERR(logger, "Error initializing NASController module");
    }

    if (!modules.insert<FlightModeManager>(new FlightModeManager()))
    {
        initResult = false;
        LOG_ERR(logger, "Error initializing FlightModeManager module");
    }

    if (!modules.insert<PinHandler>(new PinHandler()))
    {
        initResult = false;
        LOG_ERR(logger, "Error initializing PinHandler module");
    }

    if (!modules.insert<Radio>(new Radio()))
    {
        initResult = false;
        LOG_ERR(logger, "Error initializing Radio module");
    }

    if (!modules.insert<Sensors>(new Sensors()))
    {
        initResult = false;
        LOG_ERR(logger, "Error initializing Sensors module");
    }

    // used indirectly by WingController
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

#ifdef HILSimulation
    auto flightPhasesManager = HIL::getInstance().flightPhasesManager;

    flightPhasesManager->setCurrentPositionSource(
        []() {
            return TimedTrajectoryPoint{
                modules.get<NASController>()->getNasState()};
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
    if (!modules.get<FlightModeManager>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the FlightModeManager");
    }

    if (!modules.get<NASController>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the NAS algorithm");
    }

    if (!modules.get<WingController>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the WingController");
    }
    modules.get<WingController>()->setAutomatic(true);  // set the
                                                        // algorithm to sequence
    //   Start the sensors sampling
    modules.get<WingController>()->addAlgorithm(new AutomaticWingAlgorithm(
        0.1, 0.01, PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO));
    modules.get<WingController>()->selectAlgorithm(0);

    if (!modules.get<Sensors>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the sensors");
    }

    // Start the board task scheduler
    if (!BoardScheduler::getInstance().getScheduler().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the General Purpose Scheduler");
    }

    // Start the pin handler and observer
    if (!modules.get<PinHandler>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the PinHandler");
    }
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
