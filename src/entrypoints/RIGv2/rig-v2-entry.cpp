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
#include <RIGv2/Buses.h>
#include <RIGv2/Radio/Radio.h>
#include <RIGv2/Sensors/Sensors.h>
#include <RIGv2/StateMachines/GroundModeManager/GroundModeManager.h>
#include <common/Events.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/StackLogger.h>
#include <events/EventBroker.h>

using namespace Boardcore;
using namespace Common;
using namespace RIGv2;
using namespace miosix;

int main()
{

    ModuleManager &modules = ModuleManager::getInstance();
    PrintLogger logger     = Logging::getLogger("main");

    // TODO: Move this to a dedicated board scheduler
    TaskScheduler *scheduler1 = new TaskScheduler(3);
    TaskScheduler *scheduler2 = new TaskScheduler(4);

    Buses *buses           = new Buses();
    Sensors *sensors       = new Sensors(*scheduler1);
    Actuators *actuators   = new Actuators(*scheduler2);
    GroundModeManager *gmm = new GroundModeManager();
    Radio *radio           = new Radio();

    Logger &sdLogger    = Logger::getInstance();
    EventBroker &broker = EventBroker::getInstance();

    bool initResult = true;

    // Insert modules
    if (!modules.insert<Buses>(buses))
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to insert Buses");
    }

    if (!modules.insert<Sensors>(sensors))
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to insert Sensors");
    }

    if (!modules.insert<Actuators>(actuators))
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to insert Actuators");
    }

    if (!modules.insert<Radio>(radio))
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to insert Radio");
    }

    if (!modules.insert<GroundModeManager>(gmm))
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to insert GroundModeManager");
    }

    // Start modules
    if (sdLogger.testSDCard())
    {
        initResult = false;
        LOG_ERR(logger, "SD card test failed");
    }

    if (broker.start())
    {
        initResult = false;
        LOG_ERR(logger, "Failed to start EventBroker");
    }

    if (!sensors->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start Sensors module");
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

    if (!gmm->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start GroundModeManager module");
    }

    if (!scheduler1->start() || !scheduler2->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start scheduler");
    }

    if (initResult)
    {
        broker.post(FMM_INIT_OK, TOPIC_MOTOR);
        LOG_INFO(logger, "All good!");
    }
    else
    {
        broker.post(FMM_INIT_ERROR, TOPIC_MOTOR);
        LOG_ERR(logger, "Init failure!");
    }

    // Periodic statistics
    while (true)
    {
        Thread::sleep(1000);
        sdLogger.log(modules.get<Radio>()->getMavStatus());
        sdLogger.log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();
        // TODO: What the fuck is this?
        // StackLogger::getInstance().log();
    }

    return 0;
}