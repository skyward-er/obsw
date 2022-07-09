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

#include <Main/Actuators/Actuators.h>
#include <Main/BoardScheduler.h>
#include <Main/Radio/Radio.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/ADAController/ADAController.h>
#include <Main/StateMachines/AirBrakesController/AirBrakesController.h>
#include <Main/StateMachines/Deployment/Deployment.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Main/StateMachines/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Main/StateMachines/NASController/NASController.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <events/EventBroker.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;
using namespace Main;

void print()
{
    auto state = NASController::getInstance().getNasState();
    printf("w%fwa%fab%fbc%fc\n", state.qw, state.qx, state.qy, state.qz);

    // printf("%f\n",
    // Sensors::getInstance().getADS131M04LastSample().voltage[0]);
}

int main()
{
    Logger::getInstance().start();
    EventBroker::getInstance().start();

    // Initialize the servo outputs
    Actuators::getInstance().enableServo(AIRBRAKES_SERVO);
    Actuators::getInstance().enableServo(EXPULSION_SERVO);

    // Start the radio
    Radio::getInstance().start();

    // TODO: Start the pin observer

    // Start the state machines
    ADAController::getInstance().start();
    AirBrakesController::getInstance().start();
    Deployment::getInstance().start();
    FlightModeManager::getInstance().start();
    FlightStatsRecorder::getInstance().start();
    NASController::getInstance().start();

    // Start the sensors sampling
    Sensors::getInstance().start();

    // DEBUG PRINT
    BoardScheduler::getInstance().getScheduler().addTask(print, 100);

    // Start the board task scheduler
    BoardScheduler::getInstance().getScheduler().start();

    // Periodically statistics
    while (true)
    {
        Thread::sleep(1000);
        Logger::getInstance().log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();
        Logger::getInstance().logStats();
        Radio::getInstance().logStatus();
    }
}
