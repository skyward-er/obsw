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

#define EIGEN_RUNTIME_NO_MALLOC  // enable eigen malloc usage assert

#include <Common.h>
#include <events/EventBroker.h>
#include <events/Events.h>
#include <events/utils/EventCounter.h>

#include "TimestampTimer.h"
#include "miosix.h"
#include "scheduler/TaskScheduler.h"
#include "sensors/SensorManager.h"

/* HIL includes */
#include "hardware_in_the_loop/HILConfig.h"
#include "hardware_in_the_loop/HIL_actuators/HILServo.h"
#include "hardware_in_the_loop/HIL_sensors/HILBarometer.h"
#include "hardware_in_the_loop/HIL_sensors/HILGps.h"
#include "hardware_in_the_loop/HIL_sensors/HILImu.h"
#include "hardware_in_the_loop/HIL_sensors/HILNas.h"  // TODO: delete

/* Aerobrakes includes */
#include "DeathStack/AeroBrakesController/AeroBrakesController.h"

/* ADA includes */
#include <ADA/ADAController.h>

/* Navigation System includes */
//#include "NavigationSystem/NASController.h"

using namespace std;
using namespace miosix;
using namespace DeathStackBoard;
using namespace NASConfigs;

bool isSimulationRunning;

/**
 * structure that contains all the sensors used in the simulation
 */
struct StateComplete
{
    HILImu *imu;
    HILBarometer *barometer;
    HILGps *gps;
    HILNas *nas;
};

uint8_t getNextSchedulerId(TaskScheduler *s)
{
    return (s->getTaskStats()).back().id + 1;
}

void setIsSimulationRunning(bool running) { isSimulationRunning = running; }

int main()
{
    isSimulationRunning = true;

    // crash if eigen dynamically allocates memory
    Eigen::internal::set_is_malloc_allowed(false);
    TimestampTimer::enableTimestampTimer();

    // Definition of the flight phases manager
    HILFlightPhasesManager *flightPhasesManager =
        HILFlightPhasesManager::getInstance();

    flightPhasesManager->registerToFlightPhase(
        SIMULATION_STOPPED, bind(&setIsSimulationRunning, false));

    /*-------------- [TS] Task Scheduler --------------*/

    TaskScheduler scheduler;

    flightPhasesManager->registerToFlightPhase(
        SIMULATION_STOPPED, bind(&TaskScheduler::stop, &scheduler));

    /*-------------- [HILT] HILTransceiver --------------*/

    // Definition of the transceiver object
    HILTransceiver *matlab = HILTransceiver::getInstance();

    // registering the HILTransceiver in order to let him know when it has to
    // wait to the control algorithm or not
    flightPhasesManager->registerToFlightPhase(
        AEROBRAKES, bind(&HILTransceiver::setIsAerobrakePhase, matlab, true));

    flightPhasesManager->registerToFlightPhase(
        APOGEE, bind(&HILTransceiver::setIsAerobrakePhase, matlab, false));

    /*-------------- Sensors & Actuators --------------*/

    // Definition of the fake sensors for the simulation
    StateComplete state;
    state.imu       = new HILImu(matlab, N_DATA_IMU);
    state.barometer = new HILBarometer(matlab, N_DATA_BARO);
    state.gps       = new HILGps(matlab, N_DATA_GPS);

    // Definition of the fake actuators for the simulation
    HILServo servo(matlab);
    servo.init();

    /*--------------- [NAS] Navigation System ----------------*/
    // TRACE("Before NAS\n");
    NASController<HILImuData, HILBaroData, HILGpsData> nas_controller(
        *state.imu, *state.barometer, *state.gps);

    state.nas = new HILNas(matlab, N_DATA_KALM, &nas_controller);
    // TRACE("After NAS\n");
    /*--------------- [SM] Sensor Manager --------------*/

    // instantiate the sensor manager with the given scheduler
    SensorManager SM(&scheduler, {{state.imu, imuConfig},
                                  {state.barometer, baroConfig},
                                  {state.gps, gpsConfig},
                                  {state.nas, kalmConfig}});

    // registering the enabling of the sensors to the flightPhasesManager
    flightPhasesManager->registerToFlightPhase(
        SIMULATION_STARTED, bind(&SensorManager::enableAllSensors, &SM));

    /*---------------- [ADA] ADA ---------------*/
    ADAController<HILBaroData, HILGpsData> *ada_controller =
        new ADAController<HILBaroData, HILGpsData>(*state.barometer,
                                                   *state.gps);

    // setDeploymentAltitude when starting calibration
    flightPhasesManager->registerToFlightPhase(
        CALIBRATION,
        bind(&ADAController<HILBaroData, HILGpsData>::setDeploymentAltitude,
             ada_controller, deploymentAltitude));

    // setReferenceAltitude when starting calibration
    flightPhasesManager->registerToFlightPhase(
        CALIBRATION,
        bind(&ADAController<HILBaroData, HILGpsData>::setReferenceAltitude,
             ada_controller, referenceAltitude));

    // setReferenceTemperature when starting calibration
    flightPhasesManager->registerToFlightPhase(
        CALIBRATION,
        bind(&ADAController<HILBaroData, HILGpsData>::setReferenceTemperature,
             ada_controller, referenceTemperature));

    /*-------------- [CA] Control Algorithm --------------*/

    // definition of the control algorithm
    AeroBrakesController<HILNasData> aerobrakeController(*state.nas, &servo);

    /*-------------- Events --------------*/

    EventCounter counter{*sEventBroker};
    counter.subscribe(TOPIC_FLIGHT_EVENTS);
    counter.subscribe(TOPIC_ADA);
    counter.subscribe(TOPIC_NAS);

    /* INSERT THE SUBSCRIBING FOR THE EVENTBROKER (inserted in the
     * flightPhasesManager object) */

    /*-------------- Adding tasks to scheduler --------------*/

    // adding the updating of the ADA to the scheduler
    {
        TaskScheduler::function_t update_ADA{bind(
            &ADAController<HILBaroData, HILGpsData>::update, ada_controller)};

        scheduler.add(update_ADA, (uint32_t)(1000 / ADA_FREQ),
                      getNextSchedulerId(&scheduler));
    }

    // adding the updating for the navigation system
    {
        TaskScheduler::function_t update_NAS{
            bind(&NASController<HILImuData, HILBaroData, HILGpsData>::update,
                 &nas_controller)};

        scheduler.add(update_NAS, (uint32_t)(1000 / KALM_FREQ),
                      getNextSchedulerId(&scheduler));
    }

    // adding the updating of the algorithm to the scheduler
    {
        TaskScheduler::function_t update_Aerobrake{bind(
            &AeroBrakesController<HILNasData>::update, &aerobrakeController)};

        scheduler.add(update_Aerobrake, (uint32_t)(1000 / CONTROL_FREQ),
                      getNextSchedulerId(&scheduler));
    }

    /*---------- Starting threads --------*/

    matlab->start();
    ada_controller->start();
    nas_controller.start();
    aerobrakeController.start();
    sEventBroker->start();
    scheduler.start();  // started only the scheduler instead of the SM

    /*---------- Normal execution --------*/
    while (isSimulationRunning)
    {
        Thread::sleep(1000 * ADAConfigs::SAMPLING_PERIOD);
    }

    return 0;
}