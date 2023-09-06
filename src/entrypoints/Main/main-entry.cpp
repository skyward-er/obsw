/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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
#include <Main/AltitudeTrigger/AltitudeTrigger.h>
#include <Main/BoardScheduler.h>
#include <Main/Buses.h>
#include <Main/CanHandler/CanHandler.h>
#include <Main/PinHandler/PinHandler.h>
#include <Main/Radio/Radio.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/ABKController/ABKController.h>
#include <Main/StateMachines/ADAController/ADAController.h>
#include <Main/StateMachines/Deployment/Deployment.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Main/StateMachines/MEAController/MEAController.h>
#include <Main/StateMachines/NASController/NASController.h>
#include <Main/TMRepository/TMRepository.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/PrintLogger.h>
#include <diagnostic/StackLogger.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <events/EventData.h>
#include <events/utils/EventSniffer.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace Boardcore;
using namespace Main;
using namespace Common;

int main()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Overall status, if at some point it becomes false, there is a problem
    // somewhere
    bool initResult    = true;
    PrintLogger logger = Logging::getLogger("main");

    // Create modules
    BoardScheduler* scheduler = new BoardScheduler();
    Buses* buses              = new Buses();
    Sensors* sensors =
        new Sensors(scheduler->getScheduler(miosix::PRIORITY_MAX - 1));
    NASController* nas =
        new NASController(scheduler->getScheduler(miosix::PRIORITY_MAX));
    ADAController* ada =
        new ADAController(scheduler->getScheduler(miosix::PRIORITY_MAX));
    Radio* radio = new Radio(scheduler->getScheduler(miosix::PRIORITY_MAX - 2));
    TMRepository* tmRepo = new TMRepository();
    CanHandler* canHandler =
        new CanHandler(scheduler->getScheduler(miosix::PRIORITY_MAX - 2));
    FlightModeManager* fmm = new FlightModeManager();
    Actuators* actuators =
        new Actuators(scheduler->getScheduler(miosix::MAIN_PRIORITY));
    Deployment* dpl        = new Deployment();
    PinHandler* pinHandler = new PinHandler();
    AltitudeTrigger* altitudeTrigger =
        new AltitudeTrigger(scheduler->getScheduler(miosix::PRIORITY_MAX));
    MEAController* mea =
        new MEAController(scheduler->getScheduler(miosix::PRIORITY_MAX));
    ABKController* abk =
        new ABKController(scheduler->getScheduler(miosix::PRIORITY_MAX));

    // Insert modules
    if (!modules.insert<BoardScheduler>(scheduler))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Board Scheduler module");
    }

    if (!modules.insert<Buses>(buses))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Buses module");
    }

    if (!modules.insert<Sensors>(sensors))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Sensor module");
    }

    if (!modules.insert<Actuators>(actuators))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Actuators module");
    }

    if (!modules.insert<Deployment>(dpl))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the DPL module");
    }

    if (!modules.insert<NASController>(nas))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the NAS module");
    }

    if (!modules.insert<ADAController>(ada))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the ADA module");
    }

    if (!modules.insert<MEAController>(mea))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the MEA module");
    }

    if (!modules.insert<ABKController>(abk))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the ABK controller module");
    }

    if (!modules.insert<Radio>(radio))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Radio module");
    }

    if (!modules.insert<FlightModeManager>(fmm))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the FMM module");
    }

    if (!modules.insert<TMRepository>(tmRepo))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the TMRepository module");
    }

    if (!modules.insert<CanHandler>(canHandler))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the CanHandler module");
    }

    if (!modules.insert<PinHandler>(pinHandler))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the PinHandler module");
    }

    if (!modules.insert<AltitudeTrigger>(altitudeTrigger))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Altitude Trigger module");
    }

    // Start modules
    if (!Logger::getInstance().testSDCard())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Logger module");
    }

    if (!EventBroker::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the EventBroker module");
    }

    if (!modules.get<BoardScheduler>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Board Scheduler module");
    }

    if (!modules.get<Actuators>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Actuators module");
    }

    if (!modules.get<Deployment>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Deployment module");
    }

    if (!modules.get<Sensors>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Sensors module");
    }

    if (!modules.get<NASController>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the NAS module");
    }

    if (!modules.get<ADAController>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the ADA module");
    }

    if (!modules.get<MEAController>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the MEA module");
    }

    if (!modules.get<ABKController>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the ABK controller module");
    }

    if (!modules.get<FlightModeManager>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the FMM module");
    }

    if (!modules.get<Radio>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Radio module");
    }

    if (!modules.get<CanHandler>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the CanHandler module");
    }

    if (!modules.get<PinHandler>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the PinHandler module");
    }

    if (!modules.get<AltitudeTrigger>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Altitude Trigger module");
    }

    // Log all the events
    EventSniffer sniffer(
        EventBroker::getInstance(), TOPICS_LIST,
        [](uint8_t event, uint8_t topic)
        {
            EventData ev{TimestampTimer::getTimestamp(), event, topic};
            Logger::getInstance().log(ev);
        });

    // Check the init result and launch an event
    if (initResult)
    {
        // Post OK
        EventBroker::getInstance().post(FMM_INIT_OK, TOPIC_FMM);

        // Set the LED status
        miosix::led1On();
    }
    else
    {
        EventBroker::getInstance().post(FMM_INIT_ERROR, TOPIC_FMM);
        LOG_ERR(logger, "Failed to initialize");
    }

    // Periodic statistics
    while (true)
    {
        Thread::sleep(1000);
        Logger::getInstance().log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();
        StackLogger::getInstance().log();
    }

    return 0;
}