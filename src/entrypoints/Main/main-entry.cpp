/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#define DEFAULT_STDOUT_LOG_LEVEL LOGL_ERROR

#include <Main/Actuators/Actuators.h>
#include <Main/AltitudeTrigger/AltitudeTrigger.h>
#include <Main/BoardScheduler.h>
#include <Main/Buses.h>
#include <Main/CanHandler/CanHandler.h>
#include <Main/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Main/PinHandler/PinHandler.h>
#include <Main/Radio/Radio.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/ABKController/ABKController.h>
#include <Main/StateMachines/ADAController/ADAController.h>
#include <Main/StateMachines/Deployment/Deployment.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Main/StateMachines/MEAController/MEAController.h>
#include <Main/StateMachines/NASController/NASController.h>
#include <Main/TMRepository/TMRepository.h>
#include <common/Events.h>
#include <common/Mavlink.h>
#include <common/Topics.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/PrintLogger.h>
#include <diagnostic/StackLogger.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <events/EventData.h>
#include <events/utils/EventSniffer.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#ifdef HILMain
#include <HIL.h>
#include <Main/Sensors/HILSensors.h>
#endif

using namespace Boardcore;
using namespace Main;
using namespace Common;

class MockRadio : public Radio
{
public:
    MockRadio(TaskScheduler* scheduler) : Radio(scheduler) {}

    bool start() override { return true; }

    void sendAck(const mavlink_message_t& msg) override { return; };

    void sendNack(const mavlink_message_t& msg) override { return; };

    void logStatus() override { return; };

    bool isStarted() override { return true; };
};

class MockCanHandler : public CanHandler
{
public:
    MockCanHandler(TaskScheduler* scheduler) : CanHandler(scheduler) {}

    bool start() override { return true; }

    bool isStarted() override { return true; }

    void sendEvent(Common::CanConfig::EventId event) override { return; }

    void sendCanCommand(ServosList servo, bool targetState,
                        uint32_t delay) override
    {
        ModuleManager::getInstance().get<Actuators>()->setCANServoPosition(
            servo, targetState);
        return;
    }
};

class MockPinHandler : public PinHandler
{
public:
    MockPinHandler() : PinHandler() {}

    bool start() override { return true; }

    bool isStarted() override { return true; }

    void onLaunchPinTransition(Boardcore::PinTransition transition) override {}
    void onNoseconeTransition(Boardcore::PinTransition transition) override {}
    void onCutterSenseTransition(Boardcore::PinTransition transition) override
    {
    }
    void onExpulsionSenseTransition(
        Boardcore::PinTransition transition) override
    {
    }
};

int main()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Overall status, if at some point it becomes false, there is a problem
    // somewhere
    bool initResult    = true;
    PrintLogger logger = Logging::getLogger("main");

    // Create modules
    BoardScheduler* scheduler = new BoardScheduler();
    Buses* buses              = new Buses();

#ifndef HILMain
    Sensors* sensors =
        new Sensors(scheduler->getScheduler(miosix::PRIORITY_MAX - 1));
#else
    HILSensors* sensors =
        new HILSensors(scheduler->getScheduler(miosix::PRIORITY_MAX - 1));
#endif

    NASController* nas =
        new NASController(scheduler->getScheduler(miosix::PRIORITY_MAX));
    ADAController* ada =
        new ADAController(scheduler->getScheduler(miosix::PRIORITY_MAX));
#ifndef HILMain
    Radio* radio = new Radio(scheduler->getScheduler(miosix::PRIORITY_MAX - 2));
    CanHandler* canHandler =
        new CanHandler(scheduler->getScheduler(miosix::PRIORITY_MAX - 2));
    PinHandler* pinHandler = new PinHandler();
#else
    Radio* radio =
        new MockRadio(scheduler->getScheduler(miosix::PRIORITY_MAX - 2));
    CanHandler* canHandler =
        new MockCanHandler(scheduler->getScheduler(miosix::PRIORITY_MAX - 2));
    PinHandler* pinHandler = new MockPinHandler();
#endif
    TMRepository* tmRepo   = new TMRepository();
    FlightModeManager* fmm = new FlightModeManager();
    Actuators* actuators =
        new Actuators(scheduler->getScheduler(miosix::MAIN_PRIORITY));
    Deployment* dpl = new Deployment();
    AltitudeTrigger* altitudeTrigger =
        new AltitudeTrigger(scheduler->getScheduler(miosix::PRIORITY_MAX));
    MEAController* mea =
        new MEAController(scheduler->getScheduler(miosix::PRIORITY_MAX));
    ABKController* abk =
        new ABKController(scheduler->getScheduler(miosix::PRIORITY_MAX));
    FlightStatsRecorder* recorder =
        new FlightStatsRecorder(scheduler->getScheduler(miosix::MAIN_PRIORITY));

    // Insert modules
    if (!modules.insert<BoardScheduler>(scheduler))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Board Scheduler module");
    }

    if (!modules.insert<Buses>(buses))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Buses module");
    }

#ifdef HILMain
    HIL* hil = new HIL(buses->usart2, new HILFlightPhasesManager());
    if (!modules.insert<HIL>(hil))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the HIL module");
    }
    else
    {
        LOG_INFO(logger, "Inserted the HIL module");
    }
#endif

    if (!modules.insert<Sensors>(sensors))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Sensor module");
    }
    else
    {
        LOG_INFO(logger, "Inserted the Sensor module");
    }

    if (!modules.insert<Actuators>(actuators))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Actuators module");
    }
    else
    {
        LOG_INFO(logger, "Inserted the Actuators module");
    }

    if (!modules.insert<Deployment>(dpl))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the DPL module");
    }

    if (!modules.insert<NASController>(nas))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the NAS module");
    }
    else
    {
        LOG_INFO(logger, "Inserted the NAS module");
    }

    if (!modules.insert<ADAController>(ada))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the ADA module");
    }
    else
    {
        LOG_INFO(logger, "Inserted the ADA module");
    }

    if (!modules.insert<MEAController>(mea))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the MEA module");
    }
    else
    {
        LOG_INFO(logger, "Inserted the MEA module");
    }

    if (!modules.insert<ABKController>(abk))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the ABK controller module");
    }
    else
    {
        LOG_INFO(logger, "Inserted the ABK controller module");
    }

    if (!modules.insert<Radio>(radio))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Radio module");
    }
    else
    {
        LOG_INFO(logger, "Inserted the Radio module");
    }

    if (!modules.insert<FlightModeManager>(fmm))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the FMM module");
    }
    else
    {
        LOG_INFO(logger, "Inserted the FMM module");
    }

    if (!modules.insert<TMRepository>(tmRepo))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the TMRepository module");
    }
    else
    {
        LOG_INFO(logger, "Inserted the TMRepository module");
    }

    if (!modules.insert<CanHandler>(canHandler))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the CanHandler module");
    }
    else
    {
        LOG_INFO(logger, "Inserted the CanHandler module");
    }

    if (!modules.insert<PinHandler>(pinHandler))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the PinHandler module");
    }
    else
    {
        LOG_INFO(logger, "Inserted the PinHandler module");
    }

    if (!modules.insert<AltitudeTrigger>(altitudeTrigger))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Altitude Trigger module");
    }
    else
    {
        LOG_INFO(logger, "Inserted the Altitude Trigger module");
    }

    if (!modules.insert<FlightStatsRecorder>(recorder))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Flight Stats Recorder module");
    }

    // Start modules
    if (!Logger::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Logger module");
    }
    else
    {
        LOG_INFO(logger, "Tested the Logger module");
    }

    if (!EventBroker::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the EventBroker module");
    }
    else
    {
        LOG_INFO(logger, "Started the EventBroker module");
    }

    if (!modules.get<BoardScheduler>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Board Scheduler module");
    }
    else
    {
        LOG_INFO(logger, "Started the Board Scheduler module");
    }

    if (!modules.get<Actuators>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Actuators module");
    }
    else
    {
        LOG_INFO(logger, "Started the Actuators module");
    }

    if (!modules.get<Deployment>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Deployment module");
    }

    if (!modules.get<Sensors>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Sensors module");
    }

    if (!modules.get<NASController>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the NAS module");
    }
    else
    {
        LOG_INFO(logger, "Started the NAS module");
    }

    if (!modules.get<ADAController>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the ADA module");
    }
    else
    {
        LOG_INFO(logger, "Started the ADA module");
    }

    if (!modules.get<MEAController>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the MEA module");
    }
    else
    {
        LOG_INFO(logger, "Started the MEA module");
    }

    if (!modules.get<ABKController>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the ABK controller module");
    }
    else
    {
        LOG_INFO(logger, "Started the ABK controller module");
    }

    if (!modules.get<FlightModeManager>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the FMM module");
    }
    else
    {
        LOG_INFO(logger, "Started the FMM module");
    }

    if (!modules.get<Radio>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Radio module");
    }
    else
    {
        LOG_INFO(logger, "Started the Radio module");
    }

    if (!modules.get<CanHandler>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the CanHandler module");
    }
    else
    {
        LOG_INFO(logger, "Started the CanHandler module");
    }

    if (!modules.get<PinHandler>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the PinHandler module");
    }
    else
    {
        LOG_INFO(logger, "Started the PinHandler module");
    }

    if (!modules.get<AltitudeTrigger>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Altitude Trigger module");
    }
    else
    {
        LOG_INFO(logger, "Started the Altitude Trigger module");
    }

    if (!modules.get<FlightStatsRecorder>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Flight Stats Recorder module");
    }

#ifdef HILMain
    if (!modules.get<HIL>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the HIL module");
    }
    else
    {
        LOG_INFO(logger, "Started the HIL module");
    }

    hil->flightPhasesManager->setCurrentPositionSource(
        [&]() { return Boardcore::TimedTrajectoryPoint(nas->getNasState()); });

    hil->flightPhasesManager->registerToFlightPhase(
        FlightPhases::SIMULATION_STARTED,
        [&]()
        {
            Thread::sleep(2000);
            EventBroker::getInstance().post(Events::TMTC_CALIBRATE,
                                            Topics::TOPIC_TMTC);
            Thread::sleep(4000);
            EventBroker::getInstance().post(Events::TMTC_ARM,
                                            Topics::TOPIC_TMTC);
            TRACE("ARM COMMAND SENT\n");
        });

    hil->flightPhasesManager->registerToFlightPhase(
        FlightPhases::SIM_FLYING,
        [&]()
        {
            canHandler->sendCanCommand(ServosList::MAIN_VALVE, 1, 7000);
            EventBroker::getInstance().post(Events::FLIGHT_LAUNCH_PIN_DETACHED,
                                            Topics::TOPIC_FLIGHT);

            hil->flightPhasesManager->registerToFlightPhase(
                FlightPhases::SIM_FLYING,
                [&]()
                {
                    EventBroker::getInstance().post(
                        Events::FLIGHT_MISSION_TIMEOUT, Topics::TOPIC_FLIGHT);
                });
        });

    hil->flightPhasesManager->registerToFlightPhase(FlightPhases::ARMED,
                                                    [&]() { miosix::ledOn(); });

    hil->flightPhasesManager->registerToFlightPhase(
        FlightPhases::LIFTOFF,
        [&]()
        {
            // Open Main valve when liftoff happens
            actuators->setCANServoPosition(ServosList::MAIN_VALVE, 1);
        });
#endif

    // Log all the events
    EventSniffer sniffer(
        EventBroker::getInstance(), TOPICS_LIST,
        [](uint8_t event, uint8_t topic)
        {
            EventData ev{TimestampTimer::getTimestamp(), event, topic};
            Logger::getInstance().log(ev);
        });

    // Check the init result and launch an event
    if (initResult)
    {
        // Post OK
        EventBroker::getInstance().post(FMM_INIT_OK, TOPIC_FMM);

        // Set the LED status
        LOG_INFO(logger, "All modules initialized!");
        miosix::led1On();
    }
    else
    {
        EventBroker::getInstance().post(FMM_INIT_ERROR, TOPIC_FMM);
        LOG_ERR(logger, "Failed to initialize");
    }

#ifdef HILMain
    bool simulation_started = false;

    hil->flightPhasesManager->registerToFlightPhase(
        FlightPhases::SIMULATION_STARTED, [&]() { simulation_started = true; });

    while (!simulation_started)
    {
        HILConfig::ActuatorData actuatorData;
        buses->usart2.write(&actuatorData, sizeof(HILConfig::ActuatorData));
        Thread::sleep(HILConfig::SIMULATION_PERIOD);
    }

    modules.get<BoardScheduler>()
        ->getScheduler(miosix::PRIORITY_MAX - 1)
        ->addTask(
            [&]()
            {
                Boardcore::ModuleManager& modules =
                    Boardcore::ModuleManager::getInstance();

                HILConfig::ADAStateHIL adaStateHIL(
                    modules.get<ADAController>()->getADAState(),
                    modules.get<ADAController>()->getStatus());

                HILConfig::NASStateHIL nasStateHIL(
                    modules.get<NASController>()->getNasState(),
                    modules.get<NASController>()->getStatus());

                HILConfig::AirBrakesStateHIL abkStateHIL(
                    modules.get<ABKController>()->getStatus());

                HILConfig::MEAStateHIL meaStateHIL(
                    modules.get<MEAController>()->getMEAState(),
                    modules.get<MEAController>()->getStatus());

                HILConfig::ActuatorsStateHIL actuatorsStateHIL{
                    actuators->getServoPosition(ServosList::AIR_BRAKES_SERVO),
                    actuators->getServoPosition(ServosList::EXPULSION_SERVO),
                    actuators->getServoPosition(ServosList::MAIN_VALVE),
                    actuators->getServoPosition(ServosList::VENTING_VALVE)};

                HILConfig::ActuatorData actuatorData(adaStateHIL, nasStateHIL,
                                                     abkStateHIL, meaStateHIL,
                                                     actuatorsStateHIL, fmm);

                // Actually sending the feedback to the simulator
                modules.get<HIL>()->send(actuatorData);
            },
            HILConfig::SIMULATION_PERIOD, TaskScheduler::Policy::RECOVER);
#endif

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
