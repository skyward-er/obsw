/* Copyright (c) 2020-2021 Skyward Experimental Rocketry
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

#define EIGEN_NO_MALLOC  // enable eigen malloc usage assert

#include <events/EventBroker.h>
#include <events/Events.h>
#include <events/utils/EventCounter.h>

#include "TimestampTimer.h"
#include "miosix.h"
#include "scheduler/TaskScheduler.h"
#include "sensors/SensorManager.h"

/* HIL includes */
#include "hardware_in_the_loop/HIL.h"
#include "hardware_in_the_loop/HIL_algorithms/HILMockKalman.h"

/* DeathStack includes */
#include "ADA/ADAController.h"
#include "AirBrakesController/AirBrakesController.h"
#include "DeploymentController/DeploymentController.h"
#include "FlightModeManager/FlightModeManager.h"
#include "NavigationSystem/NASController.h"
#include "PinHandler/PinHandler.h"
#include "diagnostic/CpuMeter.h"

using namespace std;
using namespace miosix;
using namespace DeathStackBoard;

/**
 * structure that contains all the sensors used in the simulation
 */
struct Sensors
{
    HILImu* imu;
    HILBarometer* barometer;
    HILGps* gps;
};

uint8_t getNextSchedulerId(TaskScheduler* s)
{
    return (s->getTaskStats()).back().id + 1;
}

void threadFunc(void* arg)
{
    UNUSED(arg);

    /*-------------- [FPM] Flight Phases Manager --------------*/
    HILFlightPhasesManager* flightPhasesManager =
        HIL::getInstance().flightPhasesManager;

    /*flightPhasesManager->registerToFlightPhase(
        SIMULATION_STOPPED, bind(&setIsSimulationRunning, false));*/

    /*-------------- [TS] Task Scheduler --------------*/

    TaskScheduler scheduler;

    flightPhasesManager->registerToFlightPhase(
        SIMULATION_STOPPED, bind(&TaskScheduler::stop, &scheduler));

    /*-------------- [HIL] HILTransceiver --------------*/
    HILTransceiver* simulator = HIL::getInstance().imulator;

    /*-------------- Sensors & Actuators --------------*/

    // Definition of the fake sensors for the simulation
    Sensors sensors;
    sensors.imu       = new HILImu(simulator, N_DATA_IMU);
    sensors.barometer = new HILBarometer(simulator, N_DATA_BARO);
    sensors.gps       = new HILGps(simulator, N_DATA_GPS);

    /*-------------- [SM] Sensor Manager --------------*/

    // instantiate the sensor manager with the given scheduler
    SensorManager SM(&scheduler, {{sensors.imu, imuConfig},
                                  {sensors.barometer, baroConfig},
                                  {sensors.gps, gpsConfig}});

    // registering the enabling of the sensors to the flightPhasesManager
    flightPhasesManager->registerToFlightPhase(
        SIMULATION_STARTED, bind(&SensorManager::enableAllSensors, &SM));

    /*---------------- [ADA] ADA ---------------*/
    ADAController<HILBaroData, HILGpsData> ada_controller(*sensors.barometer,
                                                          *sensors.gps);

    // setDeploymentAltitude when starting the calibration
    flightPhasesManager->registerToFlightPhase(
        CALIBRATION,
        bind(&ADAController<HILBaroData, HILGpsData>::setDeploymentAltitude,
             &ada_controller, deploymentAltitude));

    // setReferenceAltitude when starting the calibration
    flightPhasesManager->registerToFlightPhase(
        CALIBRATION,
        bind(&ADAController<HILBaroData, HILGpsData>::setReferenceAltitude,
             &ada_controller, referenceAltitude));

    // setReferenceTemperature when starting the calibration
    flightPhasesManager->registerToFlightPhase(
        CALIBRATION,
        bind(&ADAController<HILBaroData, HILGpsData>::setReferenceTemperature,
             &ada_controller, referenceTemperature));

    /*---------------- [NAS] NavigationSystem ---------------*/
    NASController<HILImuData, HILBaroData, HILGpsData> nas_controller(
        *sensors.imu, *sensors.barometer, *sensors.gps);

    HIL::getInstance().etNAS(&nas_controller.getNAS());

    /*-------------- [CA] Control Algorithm --------------*/
    AirBrakesController<NASData> airbrakes_controller(nas_controller.getNAS());

    /*-------------- [DPL] Deployment Controller --------------*/
    DeploymentController dpl_controller;

    /*-------------- PinHandler --------------*/
    PinHandler pin_handler;

    /*-------------- Events --------------*/

    EventCounter counter{sEventBroker};
    counter.subscribe(TOPIC_FLIGHT_EVENTS);
    counter.subscribe(TOPIC_ADA);
    counter.subscribe(TOPIC_NAS);

    /* INSERT THE SUBSCRIBING FOR THE EVENTBROKER (inserted in the
     * flightPhasesManager object) */

    /*-------------- Adding tasks to scheduler --------------*/

    // adding the updating of the NAS to the scheduler
    {
        TaskScheduler::function_t update_NAS{
            bind(&NASController<HILImuData, HILBaroData, HILGpsData>::update,
                 &nas_controller)};

        scheduler.add(update_NAS, (uint32_t)(1000 / KALMAN_FREQ),
                      getNextSchedulerId(&scheduler));
    }

    // adding the updating of the ADA to the scheduler
    {
        TaskScheduler::function_t update_ADA{bind(
            &ADAController<HILBaroData, HILGpsData>::update, &ada_controller)};

        scheduler.add(update_ADA, (uint32_t)(1000 / ADA_FREQ),
                      getNextSchedulerId(&scheduler));
    }

    // adding the updating of the airbrakes algorithm to the scheduler
    {
        TaskScheduler::function_t update_Airbrake{
            bind(&AirBrakesController<NASData>::update, &airbrakes_controller)};

        scheduler.add(update_Airbrake, (uint32_t)(1000 / CONTROL_FREQ),
                      getNextSchedulerId(&scheduler));
    }

    /*---------- [FMM] FlightModeManager --------*/

    FlightModeManager fmm;

    /*---------- Starting threads --------*/

    sEventBroker.start();
    fmm.start();
    ada_controller.start();
    nas_controller.start();
    airbrakes_controller.start();
    // dpl_controller.start();
    // pin_handler.start();
    scheduler.start();  // started only the scheduler instead of the SM

    sEventBroker.post({EV_INIT_OK}, TOPIC_FMM);
    Thread::sleep(100);

    HIL::getInstance().tart();  // wait for first message from simulator

    /*---------- Wait for simulator to startup ----------*/
    while (!flightPhasesManager->isSimulationStarted())
        ;

    /*---------- Normal execution --------*/

    while (flightPhasesManager->isSimulationRunning())
    {
        Thread::sleep(1000);
    }

    scheduler.stop();

    sEventBroker.post({EV_LANDED}, TOPIC_FLIGHT_EVENTS);

    Thread::sleep(1000);
}

int main()
{
    Thread::create(threadFunc, STACK_MIN_FOR_SKYWARD, MAIN_PRIORITY, nullptr);

    unsigned int counter = 0;
    for (;;)
    {
        if (counter == 10)
        {
            TRACE("CPU : %.2f \n", averageCpuUtilization());
            counter = 0;
        }
        else
        {
            counter++;
        }

        Thread::sleep(500);
    }

    return 0;
}