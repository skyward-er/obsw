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
#include <Main/BoardScheduler.h>
#include <Main/Buses.h>
#include <Main/CanHandler/CanHandler.h>
#include <Main/PinHandler/PinHandler.h>
#include <Main/Radio/Radio.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <actuators/Servo/Servo.h>
#include <drivers/timer/PWM.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace miosix;
using namespace Boardcore;
using namespace Main;
using namespace Common;

int main()
{
    ledOff();

    ModuleManager &modules = ModuleManager::getInstance();
    PrintLogger logger     = Logging::getLogger("main");

    Buses *buses              = new Buses();
    BoardScheduler *scheduler = new BoardScheduler();

    Actuators *actuators   = new Actuators();
    Sensors *sensors       = new Sensors();
    Radio *radio           = new Radio();
    CanHandler *canHandler = new CanHandler();
    PinHandler *pinHandler = new PinHandler();
    FlightModeManager *fmm = new FlightModeManager();

    // Insert modules
    bool initResult = modules.insert<Buses>(buses) &&
                      modules.insert<BoardScheduler>(scheduler) &&
                      modules.insert<Sensors>(sensors) &&
                      modules.insert<Radio>(radio) &&
                      modules.insert<Actuators>(actuators) &&
                      modules.insert<CanHandler>(canHandler) &&
                      modules.insert<PinHandler>(pinHandler) &&
                      modules.insert<FlightModeManager>(fmm);

    // Status led indicators
    // led1: Sensors ok
    // led2: Radio ok
    // led3: CanBus ok
    // led4: Everything ok

    if (!EventBroker::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start EventBroker");
    }

    if (!actuators->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start Sensors module");
    }

    if (!sensors->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start Sensors module");
    }
    else
    {
        led1On();
    }

    if (!radio->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start Radio module");
    }
    else
    {
        led2On();
    }

    if (!canHandler->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start CanHandler module");
    }
    else
    {
        led3On();
    }

    if (!pinHandler->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start PinHandler module");
    }

    if (!scheduler->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start scheduler");
    }

    if (!fmm->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error failed to start FlightModeManager");
    }

    if (!initResult)
    {
        LOG_ERR(logger, "Init failure!");
        EventBroker::getInstance().post(FMM_INIT_ERROR, TOPIC_FMM);
        actuators->setStatusErr();
    }
    else
    {
        LOG_INFO(logger, "All good!");
        EventBroker::getInstance().post(FMM_INIT_OK, TOPIC_FMM);
        actuators->setStatusOk();
        led4On();
    }

    for (auto info : sensors->getSensorInfos())
    {
        LOG_INFO(logger, "{} {}", info.isInitialized, info.id);
    }

    while (true)
    {
        gpios::boardLed::low();
        // actuators->setAbkPosition(0.1f);
        canHandler->sendEvent(Common::CanConfig::EventId::ARM);
        Thread::sleep(1000);
        gpios::boardLed::high();
        // actuators->setAbkPosition(0.5f);
        canHandler->sendEvent(Common::CanConfig::EventId::DISARM);
        Thread::sleep(1000);
    }

    return 0;
}