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
#include <cstdio>
#include <cstdlib>

#include "Main/BoardScheduler.h"
#include "Main/Buses.h"
#include "Main/Sensors/Sensors.h"
#include "drivers/timer/TimestampTimer.h"
#include "drivers/usart/USART.h"
#include "events/EventBroker.h"
#include "kernel/scheduler/priority/priority_scheduler.h"
#include "miosix.h"

/* HIL includes */
#include "HIL.h"
#include "HIL_sensors/HILSensors.h"

/* Radio includes */
#ifdef HILUseRadio
#include <Main/Radio/Radio.h>
#endif  // HILUseRadio

/* ADA includes */
#ifdef HILUseADA
#include <Main/Configs/ADAConfig.h>
#include <Main/StateMachines/ADAController/ADAController.h>
#include <algorithms/ADA/ADA.h>
#endif  // HILUseADA

/* NAS includes */
#ifndef HILMockNAS
#include <Main/Configs/NASConfig.h>
#include <Main/StateMachines/NASController/NASController.h>
#else  // HILMockNAS
#include "HIL_algorithms/HILMockKalman.h"
#endif  // HILMockNAS

/* Airbrakes includes */
#ifndef HILMockAirBrakes
#include <Main/Configs/AirBrakesControllerConfig.h>
#include <Main/StateMachines/AirBrakesController/AirBrakesController.h>
#else  // HILMockAirBrakes
#include "HIL_algorithms/HILMockAerobrakeAlgorithm.h"
#endif  // HILMockAirBrakes

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
#ifndef HILMockNAS
    return TimedTrajectoryPoint(
        Main::NASController::getInstance().getNasState());
#else
    return Main::Sensors::getInstance().state.kalman->getLastSample();
#endif  // HILMockNAS
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
 *
 * Can be built with these flags:
 * HILMockNAS: uses the nas state calculated by the simulator
 * HILMockAirBrakes: uses the mock airBrakes algorithm (returns always 50%)
 * HILUseADA: uses the real ADA to calculate
 * HILUseRadio: uses the radio to send telemetry and receive telecommands
 * USE_SERIAL_TRANSCEIVER: Uses the radio with the serial transceiver
 */
int main()
{
    /*-------------- Pin initialization [to be removed] --------------*/
    // // radio
    // u2rx1::getPin().mode(miosix::Mode::ALTERNATE);
    // u2rx1::getPin().alternateFunction(7);
    // u2tx1::getPin().mode(miosix::Mode::ALTERNATE);
    // u2tx1::getPin().alternateFunction(7);

    // // hil
    // u3rx1::getPin().mode(miosix::Mode::ALTERNATE);
    // u3rx1::getPin().alternateFunction(7);
    // u3tx1::getPin().mode(miosix::Mode::ALTERNATE);
    // u3tx1::getPin().alternateFunction(7);

    /*-------------- [HILFPM] HILFlightPhasesManager --------------*/
    // Definition of the flight phases manager
    HILFlightPhasesManager* flightPhasesManager =
        HIL::getInstance().flightPhasesManager;

    flightPhasesManager->setCurrentPositionSource(getCurrentPosition);

    /*-------------- [TS] Task Scheduler --------------*/

    TaskScheduler& scheduler =
        Main::BoardScheduler::getInstance().getScheduler();

    /*---------------- [Radio] Radio ---------------*/
#ifdef HILUseRadio
    TRACE("Starting Radio\n");
    Main::Radio::getInstance().start();
#endif  // HILUseRadio

    /*-------------- [HILT] HILTransceiver --------------*/

    // [REMOVE] only in order to test the hil with the discovery
    u3rx1::getPin().mode(miosix::Mode::ALTERNATE);
    u3rx1::getPin().alternateFunction(7);
    u3tx1::getPin().mode(miosix::Mode::ALTERNATE);
    u3tx1::getPin().alternateFunction(7);

    TRACE("Starting hil\n");
    HIL::getInstance().start();

    /*-------------- [EB] Event Broker --------------*/
    TRACE("Starting event broker\n");
    EventBroker& eventBroker = EventBroker::getInstance();
    eventBroker.start();

    /*-------------- [CA] Control Algorithm --------------*/
#ifndef HILMockAirBrakes
    Main::AirBrakesController& airbrakes_controller =
        Main::AirBrakesController::getInstance();

    TaskScheduler::function_t update_Airbrake{
        bind(&Main::AirBrakesController::update, &airbrakes_controller)};

    TRACE("Starting abk\n");
    airbrakes_controller.start();
#else   // HILMockAirBrakes
    // definition of the MOCK control algorithm
    MockAirbrakeAlgorithm mockAirbrake(getCurrentPosition);

    mockAirbrake.init();
    mockAirbrake.begin();

    TaskScheduler::function_t update_Airbrake{
        bind(&MockAirbrakeAlgorithm::update, &mockAirbrake)};
#endif  // HILMockAirBrakes

    // adding to the scheduler the update of the AirBrakes
    scheduler.addTask(update_Airbrake, (uint32_t)(1000 / CONTROL_FREQ));

    /*---------------- [ADA] ADA ---------------*/
#ifdef HILUseADA
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
#endif  // HILUseADA

/*---------------- [NAS] NAS ---------------*/
#ifndef HILMockNAS
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

#endif  // HILMockNAS

    /*---------------- [FMM] FMM ---------------*/
    TRACE("Starting FMM\n");
    Main::FlightModeManager::getInstance().start();

    /*---------------- [Sensors] Sensors ---------------*/
    TRACE("Starting sensors\n");
    Main::Sensors::getInstance().start();

    /*---------- [DPL] Deployment --------*/
    TRACE("Starting deployment\n");
    Main::Deployment::getInstance().start();

    /*---------- [DPL] Deployment --------*/
    TRACE("Starting board TS\n");
    scheduler.start();

    /*---------- Normal execution --------*/

    // When simulation starts, we make the state machines reach the wanted state
    // (used if we don't want to use the GS to trigger these events)
    flightPhasesManager->registerToFlightPhase(
        FlightPhases::SIMULATION_STARTED,
        [&]()
        {
#ifndef HILMockNAS
            // starting NAS only when simulation starts in order to avoid
            // erroneus attitude estimation
            TRACE("Starting nas\n");
            nas_controller.start();
            Thread::sleep(500);
#endif  // HILMockNAS

#ifdef HILUseADA
            // calibrate ADA
            eventBroker.post(ADA_CALIBRATE, TOPIC_ADA);
            Thread::sleep(50);
#endif  // HILUseADA

#ifndef HILMockNAS
            eventBroker.post(NAS_CALIBRATE, TOPIC_NAS);
            Thread::sleep(50);
#endif  // HILMockNAS

            // ask to arm the board and get ready for launch
            eventBroker.post(TMTC_ARM, TOPIC_TMTC);

            TRACE("started everything\n");

            // // To show statistics on the threads 3 seconds after armed
            // Thread::sleep(3000);
            // showThreadStackSizes();
        });

    // To show statistics on the threads 10 seconds after initialization of all
    // the components
    // Thread::sleep(10000);
    // showThreadStackSizes();

    while (true)
    {
        Thread::sleep(1000);

        if (HIL::getInstance().isSimulationRunning())
        {

            TRACE("accelN: %f\n", HIL::getInstance()
                                      .simulator->getSensorData()
                                      ->accelerometer.measures[0]
                                      .getX());

#ifndef HILMockNAS
            Boardcore::NASState nasState =
                Main::NASController::getInstance().getNasState();
            TRACE("nas -> n:%+.3f, e:%+.3f  d:%+.3f\n", nasState.n, nasState.e,
                  nasState.d);
            TRACE("nas -> vn:%+.3f, ve:%+.3f  vd:%+.3f\n", nasState.vn,
                  nasState.ve, nasState.vd);
#else   // HILMockNAS
            Boardcore::TimedTrajectoryPoint point =
                Main::Sensors::getInstance().state.kalman->getLastSample();
            TRACE("point -> z:%+.3f, vz:%+.3f\n", point.z, point.vz);
#endif  // HILMockNAS

            HIL::getInstance().simulator->getSensorData()->printKalman();
            // Boardcore::ADAState adaState =
            //     Main::ADAController::getInstance().getAdaState();
            // TRACE("ada -> agl:%+.3f, vz:%+.3f\n\n", adaState.mslAltitude,
            //       adaState.verticalSpeed);
        }
    }

    return 0;
}
