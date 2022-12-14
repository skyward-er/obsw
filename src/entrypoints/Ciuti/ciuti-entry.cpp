/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <Ciuti/Algorithm/UprightDetector.h>
#include <Ciuti/BoardScheduler.h>
#include <Ciuti/Sensors/Sensors.h>
#include <Ciuti/Serial/SerialWatcher.h>
#include <diagnostic/CpuMeter/CpuMeter.h>

#include <thread>

using namespace miosix;
using namespace Boardcore;
using namespace Ciuti;

void printCiutiBanner()
{
    // clang-format off
    printf(
        R"( Booting ...                                 )""\n"
        R"( ________  ___  ___  ___  _________  ___     )""\n"
        R"(|\   ____\|\  \|\  \|\  \|\___   ___\\  \    )""\n"
        R"(\ \  \___|\ \  \ \  \\\  \|___ \  \_\ \  \   )""\n"
        R"( \ \  \    \ \  \ \  \\\  \   \ \  \ \ \  \  )""\n"
        R"(  \ \  \____\ \  \ \  \\\  \   \ \  \ \ \  \ )""\n"
        R"(   \ \_______\ \__\ \_______\   \ \__\ \ \__\)""\n"
        R"(    \|_______|\|__|\|_______|    \|__|  \|__|)""\n"
        R"(                     On Board Software       )""\n"
    );
    // clang-format on
}

int main()
{
    printCiutiBanner();

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("ciuti.main");

    Sensors::getInstance().start();
    UprightDetectorController::getInstance().start();
    SerialWatcherController::getInstance().start();

    BoardScheduler::getInstance().getScheduler().start();

    LOG_INFO(logger, "Initialization complete!");

    // Periodical statistics
    while (true)
    {
        Thread::sleep(1000);
        Logger::getInstance().log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();
        Logger::getInstance().logStats();
    }
}
