/* Copyright (c) 2024 Skyward Experimental Rocketry
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

// #define DEFAULT_STDOUT_LOG_LEVEL LOGL_WARNING
#include <Payload/Actuators/Actuators.h>
#include <Payload/AltitudeTrigger/AltitudeTrigger.h>
#include <Payload/BoardScheduler.h>
#include <Payload/Buses.h>
#include <Payload/CanHandler/CanHandler.h>
#include <Payload/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Payload/PinHandler/PinHandler.h>
#include <Payload/Radio/Radio.h>
#include <Payload/Sensors/Sensors.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Payload/StateMachines/NASController/NASController.h>
#include <Payload/StateMachines/WingController/WingController.h>
#include <Payload/TMRepository/TMRepository.h>
#include <Payload/VerticalVelocityTrigger/VerticalVelocityTrigger.h>
#include <Payload/WindEstimationScheme/WindEstimation.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/PrintLogger.h>
#include <diagnostic/StackLogger.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <events/EventData.h>
#include <events/utils/EventSniffer.h>
#include <utils/DependencyManager/DependencyManager.h>

using namespace Boardcore;
using namespace Payload;
using namespace Common;

/**
 * Starts a module and checks if it started correctly.
 * Must be followed by a semicolon or a block of code.
 * The block of code will be executed only if the module starts correctly.
 *
 * @example START_MODULE(sensors) { miosix::ledOn(); }
 */
#define START_MODULE(module)                        \
    if (!module->start())                           \
    {                                               \
        initResult = false;                         \
        LOG_ERR(logger, "Error starting " #module); \
    }                                               \
    else

int main()
{
    miosix::ledOff();

    PrintLogger logger = Logging::getLogger("Payload");
    DependencyManager depman{};

    auto buses     = new Buses();
    auto scheduler = new BoardScheduler();

    // Attitude estimation are critical components
    auto nas     = new NASController(scheduler->getCriticalScheduler());
    auto sensors = new Sensors(scheduler->getHighScheduler(), *buses);

    // Radio and CAN
    auto radio      = new Radio(scheduler->getMediumScheduler());
    auto canHandler = new CanHandler(scheduler->getMediumScheduler());

    // Flight algorithms
    auto altitudeTrigger = new AltitudeTrigger(scheduler->getMediumScheduler());
    auto wingController  = new WingController(scheduler->getMediumScheduler());
    auto verticalVelocityTrigger =
        new VerticalVelocityTrigger(scheduler->getMediumScheduler());
    auto windEstimation = new WindEstimation(scheduler->getMediumScheduler());

    // Actuators is considered non-critical since the scheduler is only used for
    // the led and buzzer tasks
    auto actuators     = new Actuators(scheduler->getLowScheduler());
    auto statsRecorder = new FlightStatsRecorder(scheduler->getLowScheduler());

    // Components without a scheduler
    auto tmRepository      = new TMRepository();
    auto flightModeManager = new FlightModeManager();
    auto pinHandler        = new PinHandler();

    // Insert modules
    bool initResult =
        depman.insert(buses) && depman.insert(scheduler) &&
        depman.insert(nas) && depman.insert(sensors) & depman.insert(radio) &&
        depman.insert(altitudeTrigger) && depman.insert(wingController) &&
        depman.insert(verticalVelocityTrigger) &&
        depman.insert(windEstimation) && depman.insert(canHandler) &&
        depman.insert(actuators) && depman.insert(statsRecorder) &&
        depman.insert(tmRepository) && depman.insert(flightModeManager) &&
        depman.insert(pinHandler);

    // Populate module dependencies
    initResult &= depman.inject();

    /* Status led indicators
    led1: Sensors ok
    led2: Radio ok
    led3: CanBus ok
    led4: Everything ok */

    // Start modules
    if (!Logger::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Logger module");
    }

    if (!EventBroker::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the EventBroker module");
    }

    START_MODULE(nas);
    START_MODULE(sensors) { miosix::led1On(); }
    START_MODULE(radio) { miosix::led2On(); }
    START_MODULE(canHandler) { miosix::led3On(); }

    START_MODULE(altitudeTrigger);
    START_MODULE(wingController);
    START_MODULE(verticalVelocityTrigger);
    START_MODULE(windEstimation);

    START_MODULE(actuators);
    START_MODULE(statsRecorder);

    START_MODULE(flightModeManager);
    START_MODULE(pinHandler);

    START_MODULE(scheduler);

    // Log all the events
    EventSniffer sniffer(
        EventBroker::getInstance(), TOPICS_LIST,
        [](uint8_t event, uint8_t topic)
        {
            EventData ev{TimestampTimer::getTimestamp(), event, topic};
            Logger::getInstance().log(ev);
        });

    if (initResult)
    {
        // Post OK
        EventBroker::getInstance().post(FMM_INIT_OK, TOPIC_FMM);

        // Set the LED status
        miosix::led4On();
    }
    else
    {
        EventBroker::getInstance().post(FMM_INIT_ERROR, TOPIC_FMM);
        LOG_ERR(logger, "Failed to initialize");
    }

    // Periodic statistics
    while (true)
    {
        Thread::sleep(1000);
        Logger::getInstance().log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();
        StackLogger::getInstance().log();
    }

    return 0;
}
