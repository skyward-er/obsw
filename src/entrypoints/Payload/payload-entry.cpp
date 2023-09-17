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

#define DEFAULT_STDOUT_LOG_LEVEL LOGL_WARNING

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

#ifdef HILMain
#include <HIL.h>
#include <Main/Sensors/HILSensors.h>
#include <Payload/Sensors/HILSensors.h>
#include <algorithms/AirBrakes/TrajectoryPoint.h>
#endif

using namespace Boardcore;
using namespace Payload;
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
};

class MockPinHandler : public PinHandler
{
public:
    MockPinHandler() : PinHandler() {}

    bool start() override { return true; }

    bool isStarted() override { return true; }

    void onExpulsionPinTransition(Boardcore::PinTransition transition) override
    {
    }
};

int main()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Overall status, if at some point it becomes false, there is a problem
    // somewhere
    bool initResult    = true;
    PrintLogger logger = Logging::getLogger("Payload");

    // Scheduler
    BoardScheduler* scheduler = new BoardScheduler();

    // Nas priority (Max priority)
    NASController* nas =
        new NASController(scheduler->getScheduler(miosix::PRIORITY_MAX));

    // Sensors priority (MAX - 1)
#ifndef HILPayload
    Sensors* sensors =
        new Sensors(scheduler->getScheduler(miosix::PRIORITY_MAX - 1));
#else
    HILSensors* sensors =
        new HILSensors(scheduler->getScheduler(miosix::PRIORITY_MAX - 1));
#endif

    // Other critical components (Max - 2)
    Radio* radio =
        new MockRadio(scheduler->getScheduler(miosix::PRIORITY_MAX - 2));
    AltitudeTrigger* altTrigger =
        new AltitudeTrigger(scheduler->getScheduler(miosix::PRIORITY_MAX - 2));
    WingController* wingController =
        new WingController(scheduler->getScheduler(miosix::PRIORITY_MAX - 2));
    VerticalVelocityTrigger* verticalVelocityTrigger =
        new VerticalVelocityTrigger(
            scheduler->getScheduler(miosix::PRIORITY_MAX - 2));
    WindEstimation* windEstimation =
        new WindEstimation(scheduler->getScheduler(miosix::PRIORITY_MAX - 2));
    CanHandler* canHandler =
        new MockCanHandler(scheduler->getScheduler(miosix::PRIORITY_MAX - 2));

    // Non critical components (Max - 3)
    // Actuators is considered non-critical since the scheduler is only used for
    // the led and buzzer tasks
    Actuators* actuators =
        new Actuators(scheduler->getScheduler(miosix::PRIORITY_MAX - 3));
    FlightStatsRecorder* statesRecorder = new FlightStatsRecorder(
        scheduler->getScheduler(miosix::PRIORITY_MAX - 3));

    // Components without a scheduler
    TMRepository* tmRepo   = new TMRepository();
    FlightModeManager* fmm = new FlightModeManager();
    Buses* buses           = new Buses();
    PinHandler* pinHandler = new MockPinHandler();

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

#ifdef HILPayload
    HIL* hil = new HIL(buses->usart2);
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

    if (!modules.insert<NASController>(nas))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the NAS module");
    }

    if (!modules.insert<FlightModeManager>(fmm))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the FMM module");
    }
    if (!modules.insert<Radio>(radio))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Radio module");
    }

    if (!modules.insert<TMRepository>(tmRepo))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the TMRepository module");
    }

    if (!modules.insert<Actuators>(actuators))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Actuators module");
    }

    if (!modules.insert<AltitudeTrigger>(altTrigger))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Altitude Trigger module");
    }

    if (!modules.insert<WingController>(wingController))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the WingController module");
    }

    if (!modules.insert<WindEstimation>(windEstimation))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the WindEstimation module");
    }

    if (!modules.insert<PinHandler>(pinHandler))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the PinHandler module");
    }

    if (!modules.insert<VerticalVelocityTrigger>(verticalVelocityTrigger))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the VerticalVelocityTrigger module");
    }

    if (!modules.insert<CanHandler>(canHandler))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the CanHandler module");
    }

    if (!modules.insert<FlightStatsRecorder>(statesRecorder))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the FlightStatsRecorder module");
    }

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

    if (!modules.get<FlightModeManager>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the FMM module");
    }

    if (!modules.get<Radio>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Radio module");
    }

    if (!modules.get<Actuators>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Actuators module");
    }

    if (!modules.get<AltitudeTrigger>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the AltitudeTrigger module");
    }

    if (!modules.get<WingController>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the WingController module");
    }

    if (!modules.get<WindEstimation>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the WindEstimation module");
    }

    if (!modules.get<PinHandler>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the PinHandler module");
    }

    if (!modules.get<VerticalVelocityTrigger>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the VerticalVelocityTrigger module");
    }

    if (!modules.get<CanHandler>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the CanHandler module");
    }

    if (!modules.get<FlightStatsRecorder>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the FlightStatsRecorder module");
    }

    if (!modules.get<BoardScheduler>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Board Scheduler module");
    }

#ifdef HILPayload
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
            printf("ARM COMMAND SENT\n");
        });

    hil->flightPhasesManager->registerToFlightPhase(
        FlightPhases::SIM_FLYING,
        [&]()
        {
            EventBroker::getInstance().post(Events::TMTC_FORCE_LAUNCH,
                                            Topics::TOPIC_FLIGHT);

            hil->flightPhasesManager->registerToFlightPhase(
                FlightPhases::SIM_FLYING,
                [&]()
                {
                    EventBroker::getInstance().post(
                        Events::FLIGHT_MISSION_TIMEOUT, Topics::TOPIC_FLIGHT);
                });

            miosix::userLed4::high();
        });

    hil->flightPhasesManager->registerToFlightPhase(
        FlightPhases::SIM_PARA1,
        [&]()
        {
            // FLIGHT_NC_DETACHED
            EventBroker::getInstance().post(Events::CAN_APOGEE_DETECTED,
                                            Topics::TOPIC_CAN);
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
        miosix::led1On();
    }
    else
    {
        EventBroker::getInstance().post(FMM_INIT_ERROR, TOPIC_FMM);
        LOG_ERR(logger, "Failed to initialize");
    }

#ifdef HILPayload
    // bool simulation_started = false;

    // hil->flightPhasesManager->registerToFlightPhase(
    //     FlightPhases::SIMULATION_STARTED, [&]() { simulation_started = true;
    //     });

    // while (!simulation_started)
    // {
    //     HILConfig::ActuatorData actuatorData;
    //     buses->usart2.write(&actuatorData, sizeof(HILConfig::ActuatorData));
    //     Thread::sleep(HILConfig::SIMULATION_PERIOD);
    // }

    modules.get<BoardScheduler>()
        ->getScheduler(miosix::PRIORITY_MAX - 1)
        ->addTask(
            [&]()
            {
                Boardcore::ModuleManager& modules =
                    Boardcore::ModuleManager::getInstance();

                HILConfig::NASStateHIL nasStateHIL(
                    modules.get<NASController>()->getNasState(),
                    modules.get<NASController>()->getStatus());

                HILConfig::ActuatorsStateHIL actuatorsStateHIL{
                    actuators->getServoPosition(EXPULSION_SERVO),
                    actuators->getServoPosition(AIR_BRAKES_SERVO),
                    actuators->getServoPosition(PARAFOIL_LEFT_SERVO),
                    actuators->getServoPosition(PARAFOIL_RIGHT_SERVO),
                    actuators->getServoPosition(MAIN_VALVE),
                    actuators->getServoPosition(VENTING_VALVE),
                    actuators->getServoPosition(RELEASE_VALVE),
                    actuators->getServoPosition(FILLING_VALVE),
                    actuators->getServoPosition(DISCONNECT_SERVO),
                };

                HILConfig::WESDataHIL wesData(
                    windEstimation->getWindEstimationScheme());

                HILConfig::GuidanceDataHIL guidanceData(
                    wingController->getPsiRef(), wingController->getDeltaA());

                HILConfig::ActuatorData actuatorData{
                    nasStateHIL,
                    actuatorsStateHIL,
                    wesData,
                    guidanceData,
                    hil->simulator->getSensorData()->flags,
                    fmm};

                // Actually sending the feedback to the
                // simulator
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
