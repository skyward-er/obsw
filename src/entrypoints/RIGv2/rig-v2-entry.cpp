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

#include <RIGv2/Buses.h>
#include <RIGv2/Radio/Radio.h>
#include <RIGv2/Sensors/Sensors.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/StackLogger.h>

using namespace Boardcore;
using namespace RIGv2;
using namespace miosix;

int main()
{

    ModuleManager &modules = ModuleManager::getInstance();
    PrintLogger logger     = Logging::getLogger("main");

    // TODO: Move this to a dedicated board scheduler
    TaskScheduler *scheduler = new TaskScheduler(3);

    Buses *buses     = new Buses();
    Sensors *sensors = new Sensors(*scheduler);
    Radio *radio     = new Radio();

    Logger &sdLogger = Logger::getInstance();

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

    if (!modules.insert<Radio>(radio))
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to insert Radio");
    }

    // Start modules
    if (!sensors->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start Sensors module");
    }

    if (!radio->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start Radio module");
    }

    if (!scheduler->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start scheduler");
    }

    if (!initResult)
    {
        LOG_INFO(logger, "All good!");
    }

    // TEMPORARY CODE
    // Start logger module
    sdLogger.start();

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