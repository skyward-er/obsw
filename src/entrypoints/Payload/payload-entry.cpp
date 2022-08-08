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

#include <Payload/Actuators/Actuators.h>
#include <Payload/BoardScheduler.h>
#include <Payload/CanHandler/CanHandler.h>
#include <Payload/Configs/SensorsConfig.h>
#include <Payload/FlightModeManager/FlightModeManager.h>
#include <Payload/NASController/NASController.h>
#include <Payload/Radio/Radio.h>
#include <Payload/Sensors/Sensors.h>
#include <Payload/Wing/AutomaticWingAlgorithm.h>
#include <Payload/Wing/WingController.h>
#include <common/events/Events.h>
#include <common/events/Topics.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/PrintLogger.h>
#include <events/EventBroker.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;
using namespace Payload;
using namespace Common;

int main()
{
    // The final result needed to trigger the FSM
    bool initResult = true;

    // The printlogger
    PrintLogger logger = Logging::getLogger("PayloadEntry");

    if (!Logger::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting SD logger");
    }

    if (!EventBroker::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the EventBroker");
    }

    // Initialize the servo outputs
    if (!Actuators::getInstance().enableServo(PARAFOIL_SERVO1) ||
        !Actuators::getInstance().setServo(PARAFOIL_SERVO1, 0) ||
        !Actuators::getInstance().enableServo(PARAFOIL_SERVO2) ||
        !Actuators::getInstance().setServo(PARAFOIL_SERVO2, 0))
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Actuators");
    }

    // Start the FMM
    if (!FlightModeManager::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the FMM");
    }

    // Start the radio
    if (!Radio::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the radio");
    }

    // Start the sensors sampling
    if (!Sensors::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the sensors");
    }

    // Start algorithms
    if (!NASController::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the NAS algorithm");
    }

    // Start the can interface
    if (!CanHandler::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the CAN interface");
    }

    // Start the board task scheduler
    if (!BoardScheduler::getInstance().getScheduler().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the General Purpose Scheduelr");
    }

    // Set up the wing controller
    WingController::getInstance().addAlgorithm(new AutomaticWingAlgorithm(
        0.1, 0.01, PARAFOIL_SERVO1, PARAFOIL_SERVO2));

    // If all is correctly set up i publish the init ok
    if (initResult)
        EventBroker::getInstance().post(FMM_INIT_OK, TOPIC_FMM);

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
