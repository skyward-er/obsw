/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <Motor/Buses.h>
#include <Motor/CanHandler/CanHandler.h>
#include <Motor/Sensors/Sensors.h>
#include <diagnostic/PrintLogger.h>
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace Boardcore;
using namespace Motor;
using namespace miosix;

int main()
{
    ModuleManager &modules = ModuleManager::getInstance();
    PrintLogger logger     = Logging::getLogger("main");

    // TODO: Move this to a dedicated board scheduler
    TaskScheduler *scheduler1 = new TaskScheduler(2);
    TaskScheduler *scheduler2 = new TaskScheduler(3);

    Buses *buses           = new Buses();
    Sensors *sensors       = new Sensors(*scheduler2);
    CanHandler *canHandler = new CanHandler();

    bool initResult = true;

    // Insert modules
    if (!modules.insert<Buses>(buses))
    {
        initResult = false;
    }

    if (!modules.insert<Sensors>(sensors))
    {
        initResult = false;
    }

    if (!modules.insert<CanHandler>(canHandler))
    {
        initResult = false;
    }

    // Start modules
    if (!sensors->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start Sensors module");
    }

    if (!canHandler->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start CanHandler module");
    }

    if (initResult)
    {
        LOG_INFO(logger, "All good!");
    }
    else
    {
        LOG_ERR(logger, "Init failure!");
    }

    for (auto info : sensors->getSensorInfo())
    {
        LOG_INFO(logger, "{} {}", info.isInitialized, info.id);
    }

    while (true)
    {
        gpios::boardLed::low();
        canHandler->sendEvent(Common::CanConfig::EventId::ARM);
        Thread::sleep(1000);
        gpios::boardLed::high();
        canHandler->sendEvent(Common::CanConfig::EventId::DISARM);
        Thread::sleep(1000);
    }

    return 0;
}