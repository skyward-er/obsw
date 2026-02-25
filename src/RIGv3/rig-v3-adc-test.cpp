/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <RIGv3/BoardScheduler.h>
#include <RIGv3/Buses.h>
#include <RIGv3/Sensors/Sensors.h>
#include <diagnostic/CpuMeter/CpuMeter.h>

#include <chrono>
#include <iomanip>
#include <iostream>

using namespace std::chrono;
using namespace Boardcore;
using namespace RIGv3;
using namespace miosix;

int main()
{
    DependencyManager manager;

    auto& sdLogger = Logger::getInstance();

    auto buses     = new Buses();
    auto scheduler = new BoardScheduler();

    auto sensors = new Sensors();

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

    // Start logging when system boots
    std::cout << "Starting Logger" << std::endl;
    if (!sdLogger.start())
    {
        initResult = false;
        std::cerr << "*** Failed to start Logger ***" << std::endl;

        if (!sdLogger.testSDCard())
            std::cerr << "\tReason: SD card not present or not writable"
                      << std::endl;
        else
            std::cerr << "\tReason: Logger initialization error" << std::endl;
    }
    else
    {
        sdLogger.resetStats();
        std::cout << "Logger Ok!\n"
                  << "\tLog number: " << sdLogger.getStats().logNumber
                  << std::endl;
    }

    std::cout << "Starting Sensors" << std::endl;
    if (!sensors->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start Sensors ***" << std::endl;
    }
    else
    {
        led1On();
    }

    std::cout << "Starting BoardScheduler" << std::endl;
    if (!scheduler->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start BoardScheduler ***" << std::endl;
    }

    if (initResult)
    {
        std::cout << "All good!" << std::endl;
        led4On();
    }
    else
    {
        std::cerr << "*** Init failure ***" << std::endl;
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
        miosix::Thread::sleep(1000);

        auto sample1 = sensors->getADC1LastSample();

        // Data frame marker
        std::cout << "$" << std::fixed << std::setprecision(3)
                  << sample1.timestamp / 1e6 << " ";
        // Print as millivolts
        for (int i = 0; i < 8; i++)
            std::cout << sample1.voltage[i] * 1000.f << " ";
        std::cout << "\n";

        CpuMeterData cpuStats = CpuMeter::getCpuStats();
        CpuMeter::resetCpuStats();

        std::cout << "CPU stats: " << std::fixed << std::setprecision(2)
                  << cpuStats.mean << "% (" << cpuStats.minValue << "%, "
                  << cpuStats.maxValue << "%) over " << cpuStats.nSamples
                  << " samples, stdDev: " << cpuStats.stdDev << "\n";
    }

    return 0;
}
