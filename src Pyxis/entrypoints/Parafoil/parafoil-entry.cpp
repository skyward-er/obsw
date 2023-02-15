/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <Parafoil/Actuators/Actuators.h>
#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Configs/SensorsConfig.h>
#include <Parafoil/Configs/WingConfig.h>
#include <Parafoil/NASController/NASController.h>
#include <Parafoil/Radio/Radio.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/Wing/WingController.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <events/EventBroker.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;
using namespace Parafoil;

int main()
{
    Logger::getInstance().start();
    EventBroker::getInstance().start();

    // Initialize the servo outputs
    Actuators::getInstance().enableServo(PARAFOIL_LEFT_SERVO);
    Actuators::getInstance().enableServo(PARAFOIL_RIGHT_SERVO);

    // Start the radio
    Radio::getInstance().start();

    // Start algorithms
    NASController::getInstance().start();
    WingController::getInstance().start();

    // Start the sensors sampling
    Sensors::getInstance().start();

    // Start the board task scheduler
    BoardScheduler::getInstance().getScheduler().start();

    // Periodical statistics
    while (true)
    {
        Thread::sleep(1000);
        Logger::getInstance().log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();
        Logger::getInstance().logStats();
        Radio::getInstance().logStatus();
    }
}
