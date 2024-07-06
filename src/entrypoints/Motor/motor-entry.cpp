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

#include <Motor/Actuators/Actuators.h>
#include <Motor/BoardScheduler.h>
#include <Motor/Buses.h>
#include <Motor/CanHandler/CanHandler.h>
#include <Motor/Sensors/Sensors.h>
#include <diagnostic/PrintLogger.h>
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>
#include <utils/DependencyManager/DependencyManager.h>

using namespace Boardcore;
using namespace Motor;
using namespace miosix;

int main()
{
    PrintLogger logger = Logging::getLogger("main");
    DependencyManager manager;

    Buses *buses              = new Buses();
    BoardScheduler *scheduler = new BoardScheduler();

    Sensors *sensors       = new Sensors();
    Actuators *actuators   = new Actuators();
    CanHandler *canHandler = new CanHandler();

    bool initResult = manager.insert<Buses>(buses) &&
                      manager.insert<BoardScheduler>(scheduler) &&
                      manager.insert<Sensors>(sensors) &&
                      manager.insert<Actuators>(actuators) &&
                      manager.insert<CanHandler>(canHandler) &&
                      manager.inject();

    manager.graphviz(std::cout);

    // Start modules
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

    if (!canHandler->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start CanHandler module");
    }

    if (!scheduler->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start scheduler");
    }

    if (initResult)
    {
        canHandler->setInitStatus(2);
        LOG_INFO(logger, "All good!");
    }
    else
    {
        canHandler->setInitStatus(1);
        LOG_ERR(logger, "Init failure!");
    }

    for (auto info : sensors->getSensorInfo())
    {
        LOG_INFO(logger, "Sensor {} {}", info.id, info.isInitialized);
    }

    while (true)
    {
        gpios::boardLed::low();
        Thread::sleep(1000);
        gpios::boardLed::high();
        Thread::sleep(1000);
        LOG_INFO(logger, "Vbat {}", sensors->getBatteryVoltage().voltage);
    }

    return 0;
}