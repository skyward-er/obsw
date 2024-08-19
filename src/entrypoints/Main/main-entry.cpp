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

#include <Main/Actuators/Actuators.h>
#include <Main/AlgoReference/AlgoReference.h>
#include <Main/BoardScheduler.h>
#include <Main/Buses.h>
#include <Main/CanHandler/CanHandler.h>
#include <Main/PinHandler/PinHandler.h>
#include <Main/Radio/Radio.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/ADAController/ADAController.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Main/StateMachines/MEAController/MEAController.h>
#include <Main/StateMachines/NASController/NASController.h>
#include <Main/StatsRecorder/StatsRecorder.h>
#include <actuators/Servo/Servo.h>
#include <drivers/timer/PWM.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>
#include <events/EventData.h>
#include <events/utils/EventSniffer.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>

#include <iostream>

using namespace miosix;
using namespace Boardcore;
using namespace Main;
using namespace Common;

int main()
{
    ledOff();

    DependencyManager manager;

    Buses *buses              = new Buses();
    BoardScheduler *scheduler = new BoardScheduler();

    Actuators *actuators    = new Actuators();
    Sensors *sensors        = new Sensors();
    Radio *radio            = new Radio();
    CanHandler *canHandler  = new CanHandler();
    PinHandler *pinHandler  = new PinHandler();
    FlightModeManager *fmm  = new FlightModeManager();
    AlgoReference *ref      = new AlgoReference();
    ADAController *ada      = new ADAController();
    NASController *nas      = new NASController();
    MEAController *mea      = new MEAController();
    StatsRecorder *recorder = new StatsRecorder();

    Logger &sdLogger    = Logger::getInstance();
    EventBroker &broker = EventBroker::getInstance();

    // Setup event sniffer
    EventSniffer sniffer(
        broker,
        [&](uint8_t event, uint8_t topic)
        {
            EventData data{TimestampTimer::getTimestamp(), event, topic};
            sdLogger.log(data);
        });

    // Insert modules
    bool initResult =
        manager.insert<Buses>(buses) &&
        manager.insert<BoardScheduler>(scheduler) &&
        manager.insert<Sensors>(sensors) && manager.insert<Radio>(radio) &&
        manager.insert<Actuators>(actuators) &&
        manager.insert<CanHandler>(canHandler) &&
        manager.insert<PinHandler>(pinHandler) &&
        manager.insert<FlightModeManager>(fmm) &&
        manager.insert<AlgoReference>(ref) &&
        manager.insert<ADAController>(ada) &&
        manager.insert<NASController>(nas) &&
        manager.insert<MEAController>(mea) &&
        manager.insert<StatsRecorder>(recorder) && manager.inject();

    manager.graphviz(std::cout);

    if (!initResult)
    {
        std::cout << "Failed to inject dependencies" << std::endl;
        return -1;
    }

    // Status led indicators
    // led1: Sensors ok
    // led2: Radio ok
    // led3: CanBus ok
    // led4: Everything ok

    if (!broker.start())
    {
        initResult = false;
        std::cout << "Error failed to start EventBroker" << std::endl;
    }

    if (!actuators->start())
    {
        initResult = false;
        std::cout << "Error failed to start Actuators module" << std::endl;
    }

    if (!sensors->start())
    {
        initResult = false;
        std::cout << "Error failed to start Sensors module" << std::endl;
    }
    else
    {
        led1On();
    }

    if (!radio->start())
    {
        initResult = false;
        std::cout << "Error failed to start Radio module" << std::endl;
    }
    else
    {
        led2On();
    }

    if (!canHandler->start())
    {
        initResult = false;
        std::cout << "Error failed to start CanHandler module" << std::endl;
    }
    else
    {
        led3On();
    }

    if (!pinHandler->start())
    {
        initResult = false;
        std::cout << "Error failed to start PinHandler module" << std::endl;
    }

    if (!scheduler->start())
    {
        initResult = false;
        std::cout << "Error failed to start scheduler" << std::endl;
    }

    if (!ada->start())
    {
        initResult = false;
        std::cout << "Error failed to start ADAController" << std::endl;
    }

    if (!nas->start())
    {
        initResult = false;
        std::cout << "Error failed to start NASController" << std::endl;
    }

    if (!mea->start())
    {
        initResult = false;
        std::cout << "Error failed to start MEAController" << std::endl;
    }

    if (!fmm->start())
    {
        initResult = false;
        std::cout << "Error failed to start FlightModeManager" << std::endl;
    }

    if (!initResult)
    {
        std::cout << "Init failure" << std::endl;
        EventBroker::getInstance().post(FMM_INIT_ERROR, TOPIC_FMM);
    }
    else
    {
        std::cout << "All good!" << std::endl;
        EventBroker::getInstance().post(FMM_INIT_OK, TOPIC_FMM);
        led4On();
    }

    std::cout << "Sensor status:" << std::endl;
    for (auto info : sensors->getSensorInfos())
    {
        std::cout << "- " << info.id << " status: " << info.isInitialized
                  << std::endl;
    }

    while (true)
    {
        gpios::boardLed::low();
        Thread::sleep(1000);
        gpios::boardLed::high();
        Thread::sleep(1000);
    }

    return 0;
}