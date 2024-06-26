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

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace Boardcore;
using namespace Payload;
using namespace Common;

/**
 * Starts a module and checks if it started correctly.
 * Must be followed by a semicolon or a block of code.
 * The block of code will be executed only if the module starts correctly.
 *
 * @example START_MODULE(Sensors) { miosix::ledOn(); }
 */
#define START_MODULE(class)                                      \
    if (!modules.get<class>()->start())                          \
    {                                                            \
        initResult = false;                                      \
        LOG_ERR(logger, "Error starting the " #class " module"); \
    }                                                            \
    else

int main()
{
    miosix::ledOff();

    ModuleManager &modules = ModuleManager::getInstance();
    PrintLogger logger     = Logging::getLogger("Payload");

    auto buses     = new Buses();
    auto scheduler = new BoardScheduler();

    // Attitude estimation are critical components
    auto nas     = new NASController(scheduler->getCriticalScheduler());
    auto sensors = new Sensors(scheduler->getHighScheduler());

    // Radio and CAN
    auto radio      = new Radio(scheduler->getMediumScheduler());
    auto canHandler = new CanHandler(scheduler->getMediumScheduler());

    // Flight algorithms
    auto altTrigger     = new AltitudeTrigger(scheduler->getMediumScheduler());
    auto wingController = new WingController(scheduler->getMediumScheduler());
    auto verticalVelocityTrigger =
        new VerticalVelocityTrigger(scheduler->getMediumScheduler());
    auto windEstimation = new WindEstimation(scheduler->getMediumScheduler());

    // Actuators is considered non-critical since the scheduler is only used for
    // the led and buzzer tasks
    auto actuators      = new Actuators(scheduler->getLowScheduler());
    auto statesRecorder = new FlightStatsRecorder(scheduler->getLowScheduler());

    // Components without a scheduler
    auto tmRepo     = new TMRepository();
    auto fmm        = new FlightModeManager();
    auto pinHandler = new PinHandler();

    // Insert modules
    bool initResult =
        modules.insert(buses) && modules.insert(scheduler) &&
        modules.insert(nas) &&
        modules.insert(sensors) & modules.insert(radio) &&
        modules.insert(altTrigger) && modules.insert(wingController) &&
        modules.insert(verticalVelocityTrigger) &&
        modules.insert(windEstimation) && modules.insert(canHandler) &&
        modules.insert(actuators) && modules.insert(statesRecorder) &&
        modules.insert(tmRepo) && modules.insert(fmm) &&
        modules.insert(pinHandler);

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

    START_MODULE(NASController);
    START_MODULE(Sensors) { miosix::led1On(); }
    START_MODULE(Radio) { miosix::led2On(); }
    START_MODULE(CanHandler) { miosix::led3On(); }

    START_MODULE(AltitudeTrigger);
    START_MODULE(WingController);
    START_MODULE(VerticalVelocityTrigger);
    START_MODULE(WindEstimation);

    START_MODULE(Actuators);
    START_MODULE(FlightStatsRecorder);

    START_MODULE(FlightModeManager);
    START_MODULE(PinHandler);

    START_MODULE(BoardScheduler);

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
