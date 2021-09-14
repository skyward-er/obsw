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

#include "TimestampTimer.h"
#include "miosix.h"
#include "scheduler/TaskScheduler.h"
#include "sensors/SensorManager.h"

/* HIL includes */
#include "hardware_in_the_loop/HILConfig.h"
#include "hardware_in_the_loop/HIL_actuators/HILServo.h"
#include "hardware_in_the_loop/HIL_sensors/HILAccelerometer.h"
#include "hardware_in_the_loop/HIL_sensors/HILBarometer.h"
#include "hardware_in_the_loop/HIL_sensors/HILGps.h"
#include "hardware_in_the_loop/HIL_sensors/HILGyroscope.h"
#include "hardware_in_the_loop/HIL_sensors/HILKalman.h"
#include "hardware_in_the_loop/HIL_sensors/HILMagnetometer.h"
/*#include "hardware_in_the_loop/events/Events.h"
#include "hardware_in_the_loop/events/Topics.h"*/

/* Airbrakes includes */
#include "DeathStack/AirBrakesController/AirBrakesControllerAlgorithm.h"

/* ADA includes */
#include <ADA/ADA.h>
#include <ADA/ADAController.h>
#include <configs/ADAconfig.h>
#include <events/FSM.h>
#include <events/utils/EventCounter.h>

using namespace std;
using namespace miosix;
using namespace DeathStackBoard;

bool isSimulationRunning;

/**
 * structure that contains all the sensors used in the simulation
 */
struct StateComplete
{
    HILAccelerometer *accelerometer;
    HILBarometer *barometer;
    HILGps *gps;
    HILGyroscope *gyro;
    HILMagnetometer *magnetometer;
    HILKalman *kalman;
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
        AEROBRAKES, bind(&HILTransceiver::setIsAirbrakePhase, matlab, true));

    flightPhasesManager->registerToFlightPhase(
        APOGEE, bind(&HILTransceiver::setIsAirbrakePhase, matlab, false));

    /*-------------- Sensors & Actuators --------------*/

    // Definition of the fake sensors for the simulation
    StateComplete state;
    state.accelerometer = new HILAccelerometer(matlab, N_DATA_ACCEL);
    state.barometer     = new HILBarometer(matlab, N_DATA_BARO);
    state.gps           = new HILGps(matlab, N_DATA_GPS);
    state.gyro          = new HILGyroscope(matlab, N_DATA_GYRO);
    state.magnetometer  = new HILMagnetometer(matlab, N_DATA_MAGN);
    state.kalman        = new HILKalman(matlab, N_DATA_KALM);

    // Definition of the fake actuators for the simulation
    HILServo servo(matlab);
    servo.init();

    /*-------------- [SM] Sensor Manager --------------*/

    // instantiate the sensor manager with the given scheduler
    SensorManager SM(&scheduler, {{state.accelerometer, accelConfig},
                                  {state.barometer, baroConfig},
                                  {state.gps, gpsConfig},
                                  {state.gyro, gyroConfig},
                                  {state.magnetometer, magnConfig},
                                  {state.kalman, kalmConfig}});

    // registering the enabling of the sensors to the flightPhasesManager

    // registering the enabling of the sensors to the flightPhasesManager
    flightPhasesManager->registerToFlightPhase(
        SIMULATION_STARTED, bind(&SensorManager::enableAllSensor, &SM));

    /*---------------- [ADA] ADA ---------------*/
    ADAController<HILBaroData, HILGpsData> *ada_controller =
        new ADAController<HILBaroData, HILGpsData>(state.barometer, state.gps);

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
    AirBrakesControllerAlgorithm<HILKalmanData> airbrakeCA(state.kalman,
                                                           &servo);

    flightPhasesManager->registerToFlightPhase(
        AEROBRAKES,
        bind(&AirBrakesControllerAlgorithm<HILKalmanData>::begin, &airbrakeCA));

    flightPhasesManager->registerToFlightPhase(
        APOGEE,
        bind(&AirBrakesControllerAlgorithm<HILKalmanData>::end, &airbrakeCA));

    /*-------------- Events --------------*/

    EventCounter counter{*sEventBroker};
    counter.subscribe(TOPIC_FLIGHT_EVENTS);

    counter.subscribe(TOPIC_ADA);
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

    // adding the updating of the algorithm to the scheduler
    {
        TaskScheduler::function_t update_Airbrake{bind(
            &AirBrakesControllerAlgorithm<HILKalmanData>::update, &airbrakeCA)};

        scheduler.add(update_Airbrake, (uint32_t)(1000 / CONTROL_FREQ),
                      getNextSchedulerId(&scheduler));
    }

    // adding the Idle updating of the aperture when airbrakes are disabled
    {
        TaskScheduler::function_t update_Idle{
            bind(&HILTransceiver::setIdleActuatorData, matlab)};

        scheduler.add(update_Idle, (uint32_t)(1000 / CONTROL_FREQ),
                      getNextSchedulerId(&scheduler));
    }

    /*---------- Starting threads --------*/

    matlab->start();
    ada_controller->start();
    sEventBroker->start();
    scheduler.start();  // started only the scheduler instead of the SM

    /*---------- Normal execution --------*/

    bool apogeeReached = false;

    while (isSimulationRunning)
    {
        // [TODO] metti nel taskscheduler
        if (!apogeeReached && counter.getCount(EV_ADA_APOGEE_DETECTED) > 0)
        {
            flightPhasesManager->setFlagsFlightPhases(APOGEE, true);
            apogeeReached = true;
        }

        Thread::sleep(1000 * ADAConfigs::SAMPLING_PERIOD);
    }

    return 0;
}