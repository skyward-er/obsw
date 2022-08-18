/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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
#include <events/EventBroker.h>
#include <events/FSM.h>
#include <events/utils/EventCounter.h>

#include <cstdio>
#include <cstdlib>

#include "Main/BoardScheduler.h"
#include "Main/Sensors/Sensors.h"
#include "drivers/timer/TimestampTimer.h"
#include "drivers/usart/USART.h"
#include "miosix.h"
#include "sensors/SensorManager.h"

/* HIL includes */
#include "HIL.h"
#include "HIL_algorithms/HILMockAerobrakeAlgorithm.h"
#include "HIL_algorithms/HILMockKalman.h"
#include "HIL_sensors/HILSensors.h"

/* ADA includes */
#include <Main/Configs/ADAConfig.h>
#include <Main/StateMachines/ADAController/ADAController.h>
#include <algorithms/ADA/ADA.h>

/* NAS includes */
#include <Main/Configs/NASConfig.h>
#include <Main/StateMachines/NASController/NASController.h>

/* Airbrakes includes */
#include <Main/Configs/AirBrakesControllerConfig.h>
#include <Main/StateMachines/AirBrakesController/AirBrakesController.h>

/* FlightModeManager includes */
#include <Main/Configs/FlightModeManagerConfig.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>

/* Deployment includes */
#include <Main/Configs/DeploymentConfig.h>
#include <Main/StateMachines/Deployment/Deployment.h>

using namespace std;
using namespace miosix;
using namespace Boardcore;
using namespace Common;

bool isSimulationRunning;
Thread *t;

TimedTrajectoryPoint getCurrentPosition()
{
    return TimedTrajectoryPoint(
        Main::NASController::getInstance().getNasState());
    // return static_cast<TimedTrajectoryPoint>(
    //     Main::Sensors::getInstance().state.kalman->getLastSample());
}

void setIsSimulationRunning(bool running)
{
    isSimulationRunning = running;
    t->wakeup();
}

/**
 * Test in order to see if the framework runs properly. This test just sends
 * back a valid constant aperture of the airbrakes
 */
int main()
{
    isSimulationRunning = true;
    t                   = Thread::getCurrentThread();

    // Definition of the flight phases manager
    HILFlightPhasesManager *flightPhasesManager =
        HIL::getInstance().flightPhasesManager;

    flightPhasesManager->setCurrentPositionSource(getCurrentPosition);

    // flightPhasesManager->registerToFlightPhase(
    //     FlightPhases::SIMULATION_STOPPED, bind(&setIsSimulationRunning,
    //     false));

    /*-------------- [TS] Task Scheduler --------------*/

    TaskScheduler &scheduler =
        Main::BoardScheduler::getInstance().getScheduler();

    // flightPhasesManager->registerToFlightPhase(
    //     FlightPhases::SIMULATION_STOPPED,
    //     bind(&TaskScheduler::stop, &scheduler));

    /*-------------- [HILT] HILTransceiver --------------*/

    // [REMOVE] only in order to test the hil with the discovery
    u3rx1::getPin().mode(miosix::Mode::ALTERNATE);
    u3rx1::getPin().alternateFunction(7);
    u3tx1::getPin().mode(miosix::Mode::ALTERNATE);
    u3tx1::getPin().alternateFunction(7);

    TRACE("Starting hil\n");
    HIL::getInstance().start();

    /*-------------- Event Broker --------------*/
    TRACE("Starting event broker\n");
    EventBroker &eventBroker = EventBroker::getInstance();
    eventBroker.start();

    /*---------------- [FMM] FMM ---------------*/
    TRACE("Starting FMM\n");
    Main::FlightModeManager::getInstance().start();

    /*---------------- [ADA] ADA ---------------*/
    Main::ADAController &ada_controller = Main::ADAController::getInstance();

    ada_controller.setUpdateDataFunction(
        [](Boardcore::ADAState state)
        { HIL::getInstance().getElaboratedData()->addADAState(state); });

    // set reference values when starting calibration
    ada_controller.setReferenceValues(
        {Main::ADAConfig::DEFAULT_REFERENCE_ALTITUDE,
         Main::ADAConfig::DEFAULT_REFERENCE_PRESSURE,
         Main::ADAConfig::DEFAULT_REFERENCE_TEMPERATURE});

    TRACE("Starting ada\n");
    ada_controller.start();

    /*---------------- [NAS] NAS ---------------*/
    Main::NASController &nas_controller = Main::NASController::getInstance();

    nas_controller.setUpdateDataFunction(
        [](Boardcore::NASState state)
        { HIL::getInstance().getElaboratedData()->addNASState(state); });

    // set reference values when starting calibration
    nas_controller.setReferenceValues(
        {Main::ADAConfig::DEFAULT_REFERENCE_ALTITUDE,
         Main::ADAConfig::DEFAULT_REFERENCE_PRESSURE,
         Main::ADAConfig::DEFAULT_REFERENCE_TEMPERATURE});

    TRACE("Starting nas\n");
    nas_controller.start();

    /*-------------- [CA] Control Algorithm --------------*/
    Main::AirBrakesController &airbrakes_controller =
        Main::AirBrakesController::getInstance();

    // definition of the MOCK control algorithm
    // MockAirbrakeAlgorithm mockAirbrake(getCurrentPosition, setActuator);

    // mockAirbrake.init();
    // mockAirbrake.begin();

    // // registering the starting and ending of the algorithm in base of the
    // // phase
    // flightPhasesManager->registerToFlightPhase(
    //     FlightPhases::AEROBRAKES,
    //     bind(&MockAirbrakeAlgorithm<HILKalmanData>::begin, &mockAirbrake));

    // flightPhasesManager->registerToFlightPhase(
    //     FlightPhases::APOGEE,
    //     bind(&MockAirbrakeAlgorithm<HILKalmanData>::end, &mockAirbrake));

    TRACE("Starting abk\n");
    airbrakes_controller.start();

    /*-------------- Adding tasks to scheduler --------------*/
    // adding the updating of the algorithm to the scheduler
    {
        /* update ADA 50 Hz */
        // added in the ADAController

        /* update NAS 50 Hz  */
        // added in the NASController

        /* update AirBrakes 10 Hz*/
        TaskScheduler::function_t update_Airbrake{
            bind(&Main::AirBrakesController::update, &airbrakes_controller)};

        // TaskScheduler::function_t update_Airbrake{
        //     bind(&MockAirbrakeAlgorithm::update, &mockAirbrake)};

        scheduler.addTask(update_Airbrake, (uint32_t)(1000 / CONTROL_FREQ));

        // ONLY FOR DEBUGGING
        scheduler.addTask(
            []()
            {
                if (HIL::getInstance().isSimulationRunning())
                {
                    Boardcore::ADAState adaState =
                        Main::ADAController::getInstance().getAdaState();
                    Boardcore::NASState nasState =
                        Main::NASController::getInstance().getNasState();
                    Boardcore::TimedTrajectoryPoint point =
                        Boardcore::TimedTrajectoryPoint(nasState);

                    HIL::getInstance()
                        .simulator->getSensorData()
                        ->printKalman();
                    HIL::getInstance().simulator->getSensorData()->printGPS();
                    TRACE("nas -> n:%+.3f, e:%+.3f  d:%+.3f\n", nasState.n,
                          nasState.e, nasState.d);
                    TRACE("nas -> vn:%+.3f, ve:%+.3f  vd:%+.3f\n", nasState.vn,
                          nasState.ve, nasState.vd);
                    TRACE("point -> z:%+.3f, vz:%+.3f\n", point.z, point.vz);
                    TRACE("ada -> agl:%+.3f, vz:%+.3f\n\n",
                          adaState.mslAltitude, adaState.verticalSpeed);
                    // TRACE("press:%+.3f\n", Main::Sensors::getInstance()
                    //                            .getMS5803LastSample()
                    //                            .pressure);
                    // TRACE("pit: %f %f\n",
                    //       Boardcore::TimedTrajectoryPoint(
                    //           Main::NASController::getInstance().getNasState())
                    //           .z,
                    //       Boardcore::TimedTrajectoryPoint(
                    //           Main::NASController::getInstance().getNasState())
                    //           .vz);
                }
            },
            1000);

        flightPhasesManager->registerToFlightPhase(
            FlightPhases::SIMULATION_STARTED,
            [&]()
            {
                // following the steps to wait for the launch to begin
                // TOPIC_FLIGHT
                // TOPIC_FMM
                // TOPIC_TMTC

                // start sensors
                TRACE("Starting sensors\n");
                Main::Sensors::getInstance().start();

                // calibrate algorithms
                Thread::sleep(50);
                eventBroker.post(ADA_CALIBRATE, TOPIC_ADA);

                Thread::sleep(50);
                eventBroker.post(NAS_CALIBRATE, TOPIC_NAS);

                // ask to arm the board and get ready for launch
                Thread::sleep(50);
                eventBroker.post(TMTC_ARM, TOPIC_TMTC);

                TRACE("started everything\n");

                TRACE("Available heap %d out of %d Bytes\n",
                      miosix::MemoryProfiling::getCurrentFreeHeap(),
                      miosix::MemoryProfiling::getHeapSize());

                TRACE("Available stack %d out of %d Bytes\n",
                      miosix::MemoryProfiling::getCurrentFreeStack(),
                      miosix::MemoryProfiling::getStackSize());
            });
    }

    /*---------- Starting threads --------*/

    // initialize FMM
    TRACE("Starting deployment\n");
    Main::Deployment::getInstance().start();

    TRACE("Starting board TS\n");
    scheduler.start();

    /*---------- Normal execution --------*/
    while (true)
    {
        Thread::wait();
    }

    return 0;
}
