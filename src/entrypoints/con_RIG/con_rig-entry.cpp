/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Giacomo Caironi
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

#include <con_RIG/BoardScheduler.h>
#include <con_RIG/Buses.h>
#include <con_RIG/Buttons/Buttons.h>
#include <con_RIG/Configs/ButtonsConfig.h>
#include <con_RIG/Radio/Radio.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/PrintLogger.h>
#include <events/EventBroker.h>
#include <miosix.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace miosix;
using namespace Boardcore;
using namespace con_RIG;

int main()
{

    bool initResult        = true;
    ModuleManager& modules = ModuleManager::getInstance();
    PrintLogger logger     = Logging::getLogger("main");

    if (!modules.insert<BoardScheduler>(new BoardScheduler()))
    {
        initResult = false;
        LOG_ERR(logger, "Error initializing BoardScheduler module");
    }

    if (!modules.insert<Buses>(new Buses()))
    {
        initResult = false;
        LOG_ERR(logger, "Error initializing Buses module");
    }

    if (!modules.insert<Radio>(new Radio()))
    {
        initResult = false;
        LOG_ERR(logger, "Error initializing Radio module");
    }

    if (!modules.insert<Buttons>(new Buttons()))
    {
        initResult = false;
        LOG_ERR(logger, "Error initializing Buttons module");
    }

    if (!Logger::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting logger");
    }

    if (!EventBroker::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the EventBroker");
    }

    if (!modules.get<Radio>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the radio");
    }

    if (!modules.get<Buttons>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the buttons");
    }

    if (!modules.get<BoardScheduler>()->getScheduler().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the General Purpose Scheduler");
    }

   if (initResult)
    {
        // POST OK
        // EventBroker::getInstance().post(FMM_INIT_OK, TOPIC_FMM);
    }
    else
    {
        // POST ERROR
        // EventBroker::getInstance().post(FMM_INIT_ERROR, TOPIC_FMM);
    }    

    // Periodical statistics
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
