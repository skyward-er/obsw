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
#include <Main/Buses.h>
#include <events/EventBroker.h>
#include <events/FSM.h>
#include <events/utils/EventCounter.h>

#include <cstdio>
#include <cstdlib>

#include "Main/BoardScheduler.h"
#include "Main/Sensors/Sensors.h"
#include "drivers/timer/TimestampTimer.h"
#include "drivers/usart/USART.h"
#include "kernel/scheduler/priority/priority_scheduler.h"
#include "miosix.h"
#include "sensors/SensorManager.h"

/* HIL includes */
#include "HIL.h"
#include "HIL_algorithms/HILMockAerobrakeAlgorithm.h"
#include "HIL_algorithms/HILMockKalman.h"
#include "HIL_sensors/HILSensors.h"

/* Radio includes */
#include <Main/Radio/Radio.h>

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

/**
 * function that returns the position estimated by the NAS. To be used in the
 * HILFlightPhasesManager to retrieve z and vz for the outcomes
 */
TimedTrajectoryPoint getCurrentPosition()
{
    return TimedTrajectoryPoint(
        Main::NASController::getInstance().getNasState());
}

// /**
//  * function that given a thread returns its free stack size (copied from
//  * miosix4, sorta)
//  */
// unsigned int getAbsoluteFreeStack(miosix::Thread* t)
// {
//     const unsigned int* walk =
//         t->watermark + (WATERMARK_LEN / sizeof(unsigned int));
//     const unsigned int stackSize = t->stacksize;
//     unsigned int count           = 0;

//     while (count < stackSize && *walk == miosix::STACK_FILL)
//     {
//         // Count unused stack
//         walk++;
//         count += 4;
//     }

//     // This takes in account CTXSAVE_ON_STACK. It might underestimate
//     // the absolute free stack (by a maximum of CTXSAVE_ON_STACK) but
//     // it will never overestimate it, which is important since this
//     // member function can be used to select stack sizes.
//     if (count <= miosix::CTXSAVE_ON_STACK)
//         return 0;
//     return count - miosix::CTXSAVE_ON_STACK;
// }

// /**
//  * function that shows for each thread its free space
//  */
// void showThreadStackSizes()
// {
//     TRACE("Absolute free stack of threads\n");
//     for (auto head : miosix::PriorityScheduler::thread_list)
//     {
//         TRACE("priority\n");
//         if (head != nullptr)
//         {
//             auto t = head;
//             TRACE("abs_free: %d/%d\n", getAbsoluteFreeStack(t),
//             t->stacksize); t = t->schedData.next; for (; t != head; t =
//             t->schedData.next)
//             {
//                 TRACE("abs_free: %d/%d\n", getAbsoluteFreeStack(t),
//                       t->stacksize);
//             }
//         }
//     }

//     TRACE("finished\n");

//     TRACE("Available heap %d out of %d Bytes\n",
//           miosix::MemoryProfiling::getCurrentFreeHeap(),
//           miosix::MemoryProfiling::getHeapSize());

//     TRACE("Available stack %d out of %d Bytes\n",
//           miosix::MemoryProfiling::getCurrentFreeStack(),
//           miosix::MemoryProfiling::getStackSize());
// }

/**
 * Test in order to see if the framework runs properly. This test just sends
 * back a valid constant aperture of the airbrakes
 */
int main()
{
    /*-------------- Pin initialization [to be removed] --------------*/
    // radio
    u2rx1::getPin().mode(miosix::Mode::ALTERNATE);
    u2rx1::getPin().alternateFunction(7);
    u2tx1::getPin().mode(miosix::Mode::ALTERNATE);
    u2tx1::getPin().alternateFunction(7);

    // hil
    u3rx1::getPin().mode(miosix::Mode::ALTERNATE);
    u3rx1::getPin().alternateFunction(7);
    u3tx1::getPin().mode(miosix::Mode::ALTERNATE);
    u3tx1::getPin().alternateFunction(7);

    /*-------------- [HILFPM] HILFlightPhasesManager --------------*/
    // Definition of the flight phases manager
    HILFlightPhasesManager* flightPhasesManager =
        HIL::getInstance().flightPhasesManager;

    flightPhasesManager->setCurrentPositionSource(getCurrentPosition);

    /*-------------- [TS] Task Scheduler --------------*/

    TaskScheduler& scheduler =
        Main::BoardScheduler::getInstance().getScheduler();

    /*---------------- [Radio] Radio ---------------*/
    TRACE("Starting Radio\n");
    Main::Radio::getInstance().start();

    /*-------------- [HILT] HILTransceiver --------------*/
    TRACE("Starting hil\n");
    HIL::getInstance().start();

    /*-------------- [EB] Event Broker --------------*/
    TRACE("Starting event broker\n");
    EventBroker& eventBroker = EventBroker::getInstance();
    eventBroker.start();

    /*-------------- [CA] Control Algorithm --------------*/
    Main::AirBrakesController& airbrakes_controller =
        Main::AirBrakesController::getInstance();

    TaskScheduler::function_t update_Airbrake{
        bind(&Main::AirBrakesController::update, &airbrakes_controller)};

    // definition of the MOCK control algorithm
    // MockAirbrakeAlgorithm mockAirbrake(getCurrentPosition, setActuator);

    // mockAirbrake.init();
    // mockAirbrake.begin();

    // // registering the starting and ending of the algorithm in base of the
    // // phase
    // flightPhasesManager->registerToFlightPhase(
    //     FlightPhases::AEROBRAKES,
    //     bind(&MockAirbrakeAlgorithm<HILKalmanData>::begin,
    // &mockAirbrake));

    // flightPhasesManager->registerToFlightPhase(
    //     FlightPhases::APOGEE,
    //     bind(&MockAirbrakeAlgorithm<HILKalmanData>::end, &mockAirbrake));

    // TaskScheduler::function_t update_Airbrake{
    //     bind(&MockAirbrakeAlgorithm::update, &mockAirbrake)};

    TRACE("Starting abk\n");
    airbrakes_controller.start();

    // adding to the scheduler the update of the AirBrakes
    scheduler.addTask(update_Airbrake, (uint32_t)(1000 / CONTROL_FREQ));

    /*---------------- [ADA] ADA ---------------*/
    Main::ADAController& ada_controller = Main::ADAController::getInstance();

    // setting the function that updates the data of the ADA in the struct that
    // will be sent to the simulator
    ada_controller.setUpdateDataFunction(
        [](Boardcore::ADAState state)
        { HIL::getInstance().getElaboratedData()->addADAState(state); });

    // setting initial reference values
    ada_controller.setReferenceValues(
        {Main::ADAConfig::DEFAULT_REFERENCE_ALTITUDE,
         Main::ADAConfig::DEFAULT_REFERENCE_PRESSURE,
         Main::ADAConfig::DEFAULT_REFERENCE_TEMPERATURE});

    TRACE("Starting ada\n");
    ada_controller.start();

    /*---------------- [NAS] NAS ---------------*/
    Main::NASController& nas_controller = Main::NASController::getInstance();

    // setting the function that updates the data of the NAS in the struct that
    // will be sent to the simulator
    nas_controller.setUpdateDataFunction(
        [](Boardcore::NASState state)
        { HIL::getInstance().getElaboratedData()->addNASState(state); });

    // setting initial reference values
    nas_controller.setReferenceValues(
        {Main::ADAConfig::DEFAULT_REFERENCE_ALTITUDE,
         Main::ADAConfig::DEFAULT_REFERENCE_PRESSURE,
         Main::ADAConfig::DEFAULT_REFERENCE_TEMPERATURE});

    // starting NAS only when simulation starts

    /*---------------- [FMM] FMM ---------------*/
    TRACE("Starting FMM\n");
    Main::FlightModeManager::getInstance().start();

    /*---------------- [Sensors] Sensors ---------------*/
    TRACE("Starting sensors\n");
    Main::Sensors::getInstance().start();

    /*-------------- Adding other tasks to scheduler --------------*/
    // PRINTS FOR DEBUGGING
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
                    ->printAccelerometer();
                HIL::getInstance().simulator->getSensorData()->printKalman();
                TRACE("point -> z:%+.3f, vz:%+.3f\n", point.z, point.vz);
                TRACE("nas -> n:%+.3f, e:%+.3f  d:%+.3f\n", nasState.n,
                      nasState.e, nasState.d);
                TRACE("nas -> vn:%+.3f, ve:%+.3f  vd:%+.3f\n", nasState.vn,
                      nasState.ve, nasState.vd);
                // TRACE("ada -> agl:%+.3f, vz:%+.3f\n\n", adaState.mslAltitude,
                //       adaState.verticalSpeed);
            }
        },
        1000);

    /*---------- Starting threads --------*/

    TRACE("Starting deployment\n");
    Main::Deployment::getInstance().start();

    TRACE("Starting board TS\n");
    scheduler.start();

    /*---------- Normal execution --------*/

    // When simulation starts, we make the state machines reach the wanted state
    // (used if we don't want to use the GS to trigger these events)
    flightPhasesManager->registerToFlightPhase(
        FlightPhases::SIMULATION_STARTED,
        [&]()
        {
            // starting NAS only when simulation starts in order to avoid
            // erroneus attitude estimation
            TRACE("Starting nas\n");
            nas_controller.start();
            Thread::sleep(500);

            // calibrate algorithms
            eventBroker.post(ADA_CALIBRATE, TOPIC_ADA);
            Thread::sleep(50);

            eventBroker.post(NAS_CALIBRATE, TOPIC_NAS);
            Thread::sleep(50);

            // ask to arm the board and get ready for launch
            eventBroker.post(TMTC_ARM, TOPIC_TMTC);

            TRACE("started everything\n");

            // // To show statistics on the threads 3 seconds after armed
            // Thread::sleep(3000);
            // showThreadStackSizes();
        });

    // // To show statistics on the threads 10 seconds after initialization of
    // all
    // // the components
    // Thread::sleep(10000);
    // showThreadStackSizes();

    while (true)
    {
        Thread::wait();
    }

    return 0;
}
