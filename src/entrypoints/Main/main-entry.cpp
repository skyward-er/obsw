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
#include <Main/HIL/HIL.h>
#include <Main/PersistentVars/PersistentVars.h>
#include <Main/PinHandler/PinHandler.h>
#include <Main/Radio/Radio.h>
#include <Main/Sensors/HILSensors.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/ABKController/ABKController.h>
#include <Main/StateMachines/ADAController/ADAController.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Main/StateMachines/MEAController/MEAController.h>
#include <Main/StateMachines/NASController/NASController.h>
#include <Main/StatsRecorder/StatsRecorder.h>
#include <actuators/Servo/Servo.h>
#include <drivers/timer/PWM.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <events/EventData.h>
#include <events/utils/EventSniffer.h>
#include <hil/HIL.h>
#include <interfaces-impl/hwmapping.h>
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

    bool initResult = true;

    PersistentVars *persistentVars = new PersistentVars();
    Buses *buses                   = new Buses();
    BoardScheduler *scheduler      = new BoardScheduler();

    Sensors *sensors;
    Actuators *actuators    = new Actuators();
    Radio *radio            = new Radio();
    CanHandler *canHandler  = new CanHandler();
    PinHandler *pinHandler  = new PinHandler();
    FlightModeManager *fmm  = new FlightModeManager();
    AlgoReference *ref      = new AlgoReference();
    ADAController *ada      = new ADAController();
    NASController *nas      = new NASController();
    MEAController *mea      = new MEAController();
    ABKController *abk      = new ABKController();
    StatsRecorder *recorder = new StatsRecorder();
    MainHIL *hil            = nullptr;

    // HIL
    if (persistentVars->getHilMode())
    {
        std::cout << "MAIN SimulatorData: " << sizeof(SimulatorData)
                  << ", ActuatorData: " << sizeof(ActuatorData) << std::endl;
        hil = new MainHIL();

        initResult &= manager.insert<MainHIL>(hil);

        sensors = new HILSensors(Config::HIL::ENABLE_HW);
    }
    else
    {
        sensors = new Sensors();
    }

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
    initResult = initResult && manager.insert<PersistentVars>(persistentVars) &&
                 manager.insert<Buses>(buses) &&
                 manager.insert<BoardScheduler>(scheduler) &&
                 manager.insert<Sensors>(sensors) &&
                 manager.insert<Radio>(radio) &&
                 manager.insert<Actuators>(actuators) &&
                 manager.insert<CanHandler>(canHandler) &&
                 manager.insert<PinHandler>(pinHandler) &&
                 manager.insert<FlightModeManager>(fmm) &&
                 manager.insert<AlgoReference>(ref) &&
                 manager.insert<ADAController>(ada) &&
                 manager.insert<NASController>(nas) &&
                 manager.insert<MEAController>(mea) &&
                 manager.insert<ABKController>(abk) &&
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

    if (!abk->start())
    {
        initResult = false;
        std::cout << "Error failed to start ABKController" << std::endl;
    }

    if (!fmm->start())
    {
        initResult = false;
        std::cout << "Error failed to start FlightModeManager" << std::endl;
    }

    if (sdLogger.start())
    {
        sdLogger.resetStats();
        std::cout << "SD good!" << std::endl
                  << "Log number: " << sdLogger.getStats().logNumber
                  << std::endl;
    }
    else
    {
        initResult = false;
        std::cout << "Error failed to start SD" << std::endl;
    }

    if (persistentVars->getHilMode())
    {
        std::cout << "Starting HIL" << std::endl;
        hil->start();

        if (!Config::HIL::IS_FULL_HIL)
        {
            hil->registerToFlightPhase(
                MainFlightPhases::SHUTDOWN, [&]()
                { actuators->setCanServoOpen(ServosList::MAIN_VALVE, false); });
        }

        // If we are in hil mode, there won't be the rig to send the ignition
        // command. The Main will do it when receives the LIFTOFF command
        hil->registerToFlightPhase(MainFlightPhases::LIFTOFF,
                                   [&]()
                                   {
                                       std::cout << "LIFTOFF!" << std::endl;
                                       if (Config::HIL::IS_FULL_HIL)
                                       {
                                           canHandler->sendServoOpenCommand(
                                               ServosList::MAIN_VALVE, 7000);
                                       }
                                       else
                                       {
                                           actuators->setCanServoOpen(
                                               ServosList::MAIN_VALVE, true);
                                       }
                                   });

        std::cout << "Waiting start simulation" << std::endl;
        hil->waitStartSimulation();
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

    if (initResult)
    {
        EventBroker::getInstance().post(FMM_INIT_OK, TOPIC_FMM);
        std::cout << "All good!" << std::endl;
        led4On();
    }
    else
    {
        EventBroker::getInstance().post(FMM_INIT_ERROR, TOPIC_FMM);
        std::cout << "Init failure" << std::endl;
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