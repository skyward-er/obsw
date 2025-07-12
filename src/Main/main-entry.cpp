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

#include <chrono>
#include <iomanip>
#include <iostream>

using namespace std::chrono;
using namespace miosix;
using namespace Boardcore;
using namespace Main;
using namespace Common;

int main()
{
    ledOff();

    DependencyManager manager;

    bool initResult = true;

    Buses* buses              = new Buses();
    BoardScheduler* scheduler = new BoardScheduler();

    Sensors* sensors;
    Actuators* actuators    = new Actuators();
    Radio* radio            = new Radio();
    CanHandler* canHandler  = new CanHandler();
    PinHandler* pinHandler  = new PinHandler();
    FlightModeManager* fmm  = new FlightModeManager();
    AlgoReference* ref      = new AlgoReference();
    ADAController* ada      = new ADAController();
    NASController* nas      = new NASController();
    MEAController* mea      = new MEAController();
    ABKController* abk      = new ABKController();
    StatsRecorder* recorder = new StatsRecorder();
    MainHIL* hil            = nullptr;

    // HIL
    if (PersistentVars::getHilMode())
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

    Logger& sdLogger    = Logger::getInstance();
    EventBroker& broker = EventBroker::getInstance();

    // Setup event sniffer
    EventSniffer sniffer(broker,
                         [&](uint8_t event, uint8_t topic)
                         {
                             EventData data{TimestampTimer::getTimestamp(),
                                            event, topic};
                             sdLogger.log(data);
                         });

    // Insert modules
    initResult = initResult && manager.insert<Buses>(buses) &&
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

    // Start modules
    std::cout << "Starting EventBroker" << std::endl;
    if (!broker.start())
    {
        initResult = false;
        std::cout << "*** Failed to start EventBroker ***" << std::endl;
    }

    std::cout << "Starting Actuators" << std::endl;
    if (!actuators->start())
    {
        initResult = false;
        std::cout << "*** Failed to start Actuators ***" << std::endl;
    }

    std::cout << "Starting Radio" << std::endl;
    if (!radio->start())
    {
        initResult = false;
        std::cout << "*** Failed to start Radio ***" << std::endl;
    }
    else
    {
        led2On();
    }

    std::cout << "Starting CanHandler" << std::endl;
    if (!canHandler->start())
    {
        initResult = false;
        std::cout << "*** Failed to start CanHandler ***" << std::endl;
    }
    else
    {
        led3On();
    }

    std::cout << "Starting PinHandler" << std::endl;
    if (!pinHandler->start())
    {
        initResult = false;
        std::cout << "*** Failed to start PinHandler ***" << std::endl;
    }

    std::cout << "Starting BoardScheduler" << std::endl;
    if (!scheduler->start())
    {
        initResult = false;
        std::cout << "*** Failed to start BoardScheduler ***" << std::endl;
    }

    std::cout << "Starting ADAController" << std::endl;
    if (!ada->start())
    {
        initResult = false;
        std::cout << "*** Failed to start ADAController ***" << std::endl;
    }

    std::cout << "Starting NASController" << std::endl;
    if (!nas->start())
    {
        initResult = false;
        std::cout << "*** Failed to start NASController ***" << std::endl;
    }

    std::cout << "Starting MEAController" << std::endl;
    if (!mea->start())
    {
        initResult = false;
        std::cout << "*** Failed to start MEAController ***" << std::endl;
    }

    std::cout << "Starting ABKController" << std::endl;
    if (!abk->start())
    {
        initResult = false;
        std::cout << "*** Failed to start ABKController ***" << std::endl;
    }

    std::cout << "Starting FlightModeManager" << std::endl;
    if (!fmm->start())
    {
        initResult = false;
        std::cout << "*** Failed to start FlightModeManager ***" << std::endl;
    }

    // Start logging when system boots
    std::cout << "Starting Logger" << std::endl;
    if (!sdLogger.start())
    {
        initResult = false;
        std::cout << "*** Failed to start Logger ***" << std::endl;
    }
    else
    {
        sdLogger.resetStats();
        std::cout << "Logger Ok!\n"
                  << "\tLog number: " << sdLogger.getStats().logNumber
                  << std::endl;
    }

    if (PersistentVars::getHilMode())
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
                                           canHandler->sendServoOpenCommand(
                                               ServosList::MAIN_VALVE, 7000);
                                       else
                                           actuators->setCanServoOpen(
                                               ServosList::MAIN_VALVE, true);
                                   });

        std::cout << "Waiting start simulation" << std::endl;
        hil->waitStartSimulation();
    }

    std::cout << "Starting Sensors" << std::endl;
    if (!sensors->start())
    {
        initResult = false;
        std::cout << "*** Failed to start Sensors ***" << std::endl;
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
        std::cout << "*** Init failure ***" << std::endl;
    }

    std::string sensorConfig;
    if (Config::Sensors::USING_DUAL_MAGNETOMETER)
        sensorConfig = "LIS2MDL IN + LIS2MDL EXT";
    else
        sensorConfig = "LPS22DF + LIS2MDL IN/EXT";

    std::cout << "Sensor status (config: " << sensorConfig << "):" << std::endl;
    for (auto info : sensors->getSensorInfos())
    {
        // The period being 0 means the sensor is disabled
        auto statusStr = info.period == 0ns   ? "Disabled"
                         : info.isInitialized ? "Ok"
                                              : "Error";

        std::cout << "\t" << std::setw(20) << std::left << info.id << " "
                  << statusStr << std::endl;
    }

    std::cout << "Battery voltage: " << std::fixed << std::setprecision(2)
              << sensors->getBatteryVoltageLastSample().voltage << " V"
              << std::endl;

    while (true)
    {
        sdLogger.log(sdLogger.getStats());

        gpios::boardLed::low();
        Thread::sleep(1000);

        sdLogger.log(sdLogger.getStats());

        gpios::boardLed::high();
        Thread::sleep(1000);
    }

    return 0;
}
