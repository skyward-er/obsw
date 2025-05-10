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
#include <RIGv2/CanHandler/CanHandler.h>
#include <RIGv2/Radio/Radio.h>
#include <RIGv2/Registry/Registry.h>
#include <RIGv2/Sensors/Sensors.h>
#include <RIGv2/StateMachines/GroundModeManager/GroundModeManager.h>
#include <RIGv2/StateMachines/TARS1/TARS1.h>
#include <common/Events.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/StackLogger.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <events/EventData.h>
#include <events/utils/EventSniffer.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

using namespace std::chrono;
using namespace Boardcore;
using namespace Common;
using namespace RIGv2;
using namespace miosix;

int main()
{
    DependencyManager manager;

    Buses* buses              = new Buses();
    BoardScheduler* scheduler = new BoardScheduler();

    Sensors* sensors = new Sensors();

    bool initResult = manager.insert<Buses>(buses) &&
                      manager.insert<BoardScheduler>(scheduler) &&
                      manager.insert<Sensors>(sensors) && manager.inject();

    if (!initResult)
    {
        std::cout << "Failed to inject dependencies" << std::endl;
        return 0;
    }

    // Status led indicators
    // led1: Sensors ok
    // led4: Everything ok

    std::cout << "Starting Sensors" << std::endl;
    if (!sensors->start())
    {
        initResult = false;
        std::cout << "*** Failed to start Sensors ***" << std::endl;
    }
    else
    {
        led1On();
    }

    std::cout << "Starting BoardScheduler" << std::endl;
    if (!scheduler->start())
    {
        initResult = false;
        std::cout << "*** Failed to start BoardScheduler ***" << std::endl;
    }

    if (initResult)
    {
        std::cout << "All good!" << std::endl;
        led4On();
    }
    else
    {
        std::cout << "*** Init failure ***" << std::endl;
    }

    std::cout << "Sensor status:" << std::endl;
    for (auto info : sensors->getSensorInfos())
    {
        // The period being 0 means the sensor is disabled
        auto statusStr = info.period == 0ns   ? "Disabled"
                         : info.isInitialized ? "Ok"
                                              : "Error";

        std::cout << "\t" << std::setw(20) << std::left << info.id << " "
                  << statusStr << std::endl;
    }

    /*
     * Print ADC data to stdout (serial port)
     *
     * Data format:
     * - ASCII mode
     * - Column delimiter: space
     * - Prefix: $ (filter by prefix)
     */
    while (true)
    {
        miosix::Thread::sleep(50);

        auto sample1 = sensors->getADC1LastSample();
        auto sample2 = sensors->getADC2LastSample();

        // Data frame marker
        std::cout << "$";
        // Print as millivolts
        for (int i = 0; i < 8; i++)
            std::cout << sample1.voltage[i] * 1000.f << " ";
        for (int i = 0; i < 8; i++)
            std::cout << sample2.voltage[i] * 1000.f << " ";
        std::cout << "\n";
    }

    return 0;
}
