/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

#include <RIGv2/Actuators/Actuators.h>
#include <RIGv2/BoardScheduler.h>
#include <RIGv2/Buses.h>
#include <RIGv2/Radio/Radio.h>
#include <RIGv2/Registry/Registry.h>
#include <RIGv2/Sensors/Sensors.h>
#include <RIGv2/StateMachines/GroundModeManager/GroundModeManager.h>
#include <RIGv2/StateMachines/TARS1/TARS1.h>
#include <common/Events.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/StackLogger.h>
#include <events/EventBroker.h>
#include <events/EventData.h>
#include <events/utils/EventSniffer.h>
// TODO(davide.mor): Remove TimestampTimer
#include <drivers/timer/TimestampTimer.h>

using namespace Boardcore;
using namespace Common;
using namespace RIGv2;
using namespace miosix;

int main()
{
    PrintLogger logger     = Logging::getLogger("main");
    ModuleManager &modules = ModuleManager::getInstance();

    Buses *buses              = new Buses();
    BoardScheduler *scheduler = new BoardScheduler();

    Sensors *sensors       = new Sensors(scheduler->getSensorsScheduler());
    Actuators *actuators   = new Actuators(scheduler->getActuatorsScheduler());
    Registry *registry     = new Registry();
    GroundModeManager *gmm = new GroundModeManager();
    TARS1 *tars1           = new TARS1(scheduler->getTars1Scheduler());
    Radio *radio           = new Radio();

    Logger &sdLogger    = Logger::getInstance();
    EventBroker &broker = EventBroker::getInstance();

    // Setup event sniffer
    EventSniffer sniffer(
        broker,
        [&](uint8_t event, uint8_t topic)
        {
            EventData data{TimestampTimer::getTimestamp(), event, topic};
            sdLogger.log(data);
        });

    // Insert modules
    bool initResult =
        modules.insert<Buses>(buses) &&
        modules.insert<BoardScheduler>(scheduler) &&
        modules.insert<Actuators>(actuators) &&
        modules.insert<Sensors>(sensors) && modules.insert<Radio>(radio) &&
        modules.insert<Registry>(registry) &&
        modules.insert<GroundModeManager>(gmm) && modules.insert<TARS1>(tars1);

    // Start modules
    if (!sdLogger.testSDCard())
    {
        initResult = false;
        LOG_ERR(logger, "SD card test failed");
    }

    if (!broker.start())
    {
        initResult = false;
        LOG_ERR(logger, "Failed to start EventBroker");
    }

    if (!registry->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start Registry module");
    }

    if (!actuators->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start Actuators module");
    }

    if (!radio->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start Radio module");
    }

    if (!sensors->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start Sensors module");
    }

    if (!gmm->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start GroundModeManager module");
    }

    if (!tars1->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start TARS1 module");
    }

    if (!scheduler->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start scheduler");
    }

    if (!initResult)
    {
        broker.post(FMM_INIT_ERROR, TOPIC_MOTOR);
        LOG_ERR(logger, "Init failure!");
    }
    else
    {
        broker.post(FMM_INIT_OK, TOPIC_MOTOR);
        LOG_INFO(logger, "All good!");
    }

    // Periodic statistics
    while (true)
    {
        Thread::sleep(1000);
        sdLogger.log(sdLogger.getStats());
        sdLogger.log(modules.get<Radio>()->getMavStatus());
        sdLogger.log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();
        // TODO: What the fuck is this?
        // StackLogger::getInstance().log();
    }

    return 0;
}