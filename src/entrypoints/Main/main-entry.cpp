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

#include <Main/Buses.h>
#include <Main/CanHandler/CanHandler.h>
#include <Main/Radio/Radio.h>
#include <Main/Sensors/Sensors.h>
#include <drivers/timer/PWM.h>
#include <miosix.h>
#include <interfaces-impl/hwmapping.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace miosix;
using namespace Boardcore;
using namespace Main;

int main()
{
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Main");
    ModuleManager &modules        = ModuleManager::getInstance();

    TaskScheduler scheduler(2);

    Buses *buses           = new Buses();
    Sensors *sensors       = new Sensors(scheduler);
    Radio *radio           = new Radio();
    CanHandler *canHandler = new CanHandler();

    if (!modules.insert<Buses>(buses))
    {
    }

    if (!modules.insert<Sensors>(sensors))
    {
    }

    if (!modules.insert<Radio>(radio))
    {
    }

    if (!modules.insert<CanHandler>(canHandler))
    {
    }

    if (!sensors->start())
    {
        LOG_ERR(logger, "Failed to init sensors");
    }
    else
    {
        LOG_INFO(logger, "Sensors init success!");
    }

    if (!radio->start())
    {
        LOG_ERR(logger, "Failed to init radio");
    }
    else
    {
        LOG_INFO(logger, "Radio init success!");
    }

    if (!canHandler->start())
    {
        LOG_ERR(logger, "Failed to init can handler");
    }
    else
    {
        LOG_INFO(logger, "Can handler init success!");
    }

    scheduler.start();

    // for(auto &info : sensors->getSensorInfo()) {
    // }

    while (true)
    {
        gpios::boardLed::high();
        Thread::sleep(1000);
        gpios::boardLed::low();
        Thread::sleep(1000);
    }

    return 0;
}