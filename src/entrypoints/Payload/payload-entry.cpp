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

// #define DEFAULT_STDOUT_LOG_LEVEL LOGL_WARNING
#include <Payload/Actuators/Actuators.h>
#include <Payload/AltitudeTrigger/AltitudeTrigger.h>
#include <Payload/BoardScheduler.h>
#include <Payload/Buses.h>
#include <Payload/CanHandler/CanHandler.h>
#include <Payload/Configs/HILSimulationConfig.h>
#include <Payload/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Payload/PinHandler/PinHandler.h>
#include <Payload/Radio/Radio.h>
#include <Payload/Sensors/HILSensors.h>
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
using namespace HILConfig;

constexpr bool hilSimulationActive = true;

int main()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Overall status, if at some point it becomes false, there is a problem
    // somewhere
    bool initResult    = true;
    PrintLogger logger = Logging::getLogger("Payload");

    // Buses
    Buses* buses = new Buses();

    // Scheduler
    BoardScheduler* scheduler = new BoardScheduler();

    // Nas priority (Max priority)
    NASController* nas =
        new NASController(scheduler->getScheduler(miosix::PRIORITY_MAX));

    // Other critical components (Max - 2)
    Radio* radio = new Radio(scheduler->getScheduler(miosix::PRIORITY_MAX - 2));
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
        new CanHandler(scheduler->getScheduler(miosix::PRIORITY_MAX - 2));

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
    PinHandler* pinHandler =
        new PinHandler(*scheduler->getScheduler(miosix::PRIORITY_MAX - 2));

    // HIL

    // Sensors priority (MAX - 1)
    Sensors* sensors;

    if (hilSimulationActive)
    {
        PayloadHILPhasesManager* hilPhasesManager = new PayloadHILPhasesManager(
            [&]()
            { return Boardcore::TimedTrajectoryPoint(nas->getNasState()); });
        PayloadHILTransceiver* hilTransceiver =
            new PayloadHILTransceiver(buses->usart2, hilPhasesManager);

        auto updateActuatorData = [&]()
        {
            Boardcore::ModuleManager& modules =
                Boardcore::ModuleManager::getInstance();

            NASStateHIL nasStateHIL(modules.get<NASController>()->getNasState(),
                                    modules.get<NASController>()->getStatus());

            ActuatorsStateHIL actuatorsStateHIL(
                actuators->getServoPosition(ServosList::PARAFOIL_LEFT_SERVO),
                actuators->getServoPosition(ServosList::PARAFOIL_RIGHT_SERVO),
                static_cast<float>(miosix::gpios::cut_trigger::value()));

            WESDataHIL wesDataHIL(windEstimation->getWindEstimationScheme());

            auto deltaA =
                actuators->getServoPosition(ServosList::PARAFOIL_LEFT_SERVO) -
                actuators->getServoPosition(ServosList::PARAFOIL_RIGHT_SERVO);

            Eigen::Vector2f heading;
            auto psiRef = wingController->emGuidance.calculateTargetAngle(
                {nasStateHIL.n, nasStateHIL.e, nasStateHIL.d}, heading);

            GuidanceDataHIL guidanceData(psiRef, deltaA, heading[0],
                                         heading[1]);

            // Returning the feedback for the simulator
            return ActuatorData(nasStateHIL, actuatorsStateHIL, wesDataHIL,
                                guidanceData, fmm);
        };

        PayloadHIL* hil = new PayloadHIL(hilTransceiver, hilPhasesManager,
                                         updateActuatorData, SIMULATION_PERIOD);

        if (!modules.insert<PayloadHIL>(hil))
        {
            initResult = false;
            LOG_ERR(logger, "Error inserting the HIL module");
        }
        else
        {
            LOG_INFO(logger, "Inserted the HIL module");
        }

        hilPhasesManager->registerToFlightPhase(
            PayloadFlightPhases::SIM_FLYING,
            [&]()
            {
                printf("Flying\n");
                EventBroker::getInstance().post(Events::CAN_LIFTOFF,
                                                Topics::TOPIC_CAN);
            });

        hilPhasesManager->registerToFlightPhase(
            PayloadFlightPhases::ARMED,
            [&]()
            {
                printf("ARMED\n");
                EventBroker::getInstance().post(Events::CAN_LIFTOFF,
                                                Topics::TOPIC_CAN);
                miosix::ledOn();
            });

        hilPhasesManager->registerToFlightPhase(
            PayloadFlightPhases::CALIBRATION_OK,
            [&]()
            {
                TRACE("ARM COMMAND SENT\n");
                EventBroker::getInstance().post(Events::TMTC_ARM,
                                                Topics::TOPIC_TMTC);
            });

        sensors =
            new HILSensors(scheduler->getScheduler(miosix::PRIORITY_MAX - 1),
                           buses, hilTransceiver, false);
    }
    else
    {

        sensors = new Sensors(scheduler->getScheduler(miosix::PRIORITY_MAX - 1),
                              buses);
    }

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
    if (hilSimulationActive)
    {
        if (!modules.get<PayloadHIL>()->start())
        {
            initResult = false;
            LOG_ERR(logger, "Error starting the HIL module");
        }

        // Waiting for start of simulation
        ModuleManager::getInstance().get<PayloadHIL>()->waitStartSimulation();
    }

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

    if (!modules.get<BoardScheduler>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Board Scheduler module");
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

    // Log all the events
    // clang-format off
    EventSniffer sniffer(EventBroker::getInstance(), TOPICS_LIST,
                         [](uint8_t event, uint8_t topic)
                         {
                             EventData ev{TimestampTimer::getTimestamp(), event,
                                          topic};
                             Logger::getInstance().log(ev);
                         });
    // clang-format on

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
