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
#include <con_RIG/Serial/Serial.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/PrintLogger.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>

#include <thread>
#include <utils/ModuleManager/ModuleManager.hpp>

using namespace miosix;
using namespace Boardcore;
using namespace ConRIG;

int main()
{
    PrintLogger logger     = Logging::getLogger("main");
    ModuleManager& modules = ModuleManager::getInstance();

    BoardScheduler* scheduler = new BoardScheduler();
    Buses* buses              = new Buses();
    Radio* radio              = new Radio(scheduler->getRadioScheduler());
    Serial* serial            = new Serial();
    Buttons* buttons          = new Buttons(scheduler->getButtonsScheduler());

    bool initResult =
        modules.insert<BoardScheduler>(scheduler) &&
        modules.insert<Buses>(buses) && modules.insert<Radio>(radio) &&
        modules.insert<Serial>(serial) && modules.insert<Buttons>(buttons);

    if (!radio->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the radio");
    }

    if (!serial->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the serial");
    }

    if (!buttons->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the buttons");
    }

    if (!scheduler->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the General Purpose Scheduler");
    }

    if (!initResult)
    {
        ui::redLed::high();
        LOG_ERR(logger, "Init failure!");
    }
    else
    {
        LOG_INFO(logger, "All good!");
    }

    // Periodical statistics
    while (true)
    {
        Thread::sleep(1000);
        CpuMeter::resetCpuStats();
    }
}
