/* Copyright (c) 2020-2022 Skyward Experimental Rocketry
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

#include "drivers/timer/TimestampTimer.h"
#include "drivers/usart/USART.h"
#include "miosix.h"
#include "scheduler/TaskScheduler.h"
#include "sensors/SensorManager.h"

/* HIL includes */
#include "HIL.h"
// #include "HIL_actuators/HILServo.h"
// #include "HIL_algorithms/HILMockAerobrakeAlgorithm.h"
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

using namespace std;
using namespace miosix;
using namespace Boardcore;

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
} state;

uint8_t getNextSchedulerId(TaskScheduler *s)
{
    return (s->getTaskStats()).back().id + 1;
}

TimedTrajectoryPoint getCurrentPosition()
{
    return static_cast<TimedTrajectoryPoint>(state.kalman->getLastSample());
}

void setIsSimulationRunning(bool running) { isSimulationRunning = running; }

void setActuator(float airbrakesOpening)
{
    HIL &hil                   = HIL::getInstance();
    ActuatorData *actuatorData = hil.getActuatorData();

    actuatorData->setAirBrakesOpening(airbrakesOpening);
    hil.simulator->setActuatorData(*actuatorData);
}

/**
 * Test in order to see if the framework runs properly. This test just sends
 * back a valid constant aperture of the airbrakes
 */
int main()
{
    isSimulationRunning = true;

    // Definition of the flight phases manager
    HILFlightPhasesManager *flightPhasesManager =
        HIL::getInstance().flightPhasesManager;

    flightPhasesManager->setCurrentPositionSource(getCurrentPosition);

    flightPhasesManager->registerToFlightPhase(
        FlightPhases::SIMULATION_STOPPED, bind(&setIsSimulationRunning, false));

    /*-------------- [TS] Task Scheduler --------------*/

    TaskScheduler scheduler;

    flightPhasesManager->registerToFlightPhase(
        FlightPhases::SIMULATION_STOPPED,
        bind(&TaskScheduler::stop, &scheduler));

    /*-------------- [HILT] HILTransceiver --------------*/

    // Definition of the transceiver object

    // [REMOVE] only in order to test the hil with the discovery
    // u3rx1::getPin().mode(miosix::Mode::ALTERNATE);
    // u3rx1::getPin().alternateFunction(7);
    // u3tx1::getPin().mode(miosix::Mode::ALTERNATE);
    // u3tx1::getPin().alternateFunction(7);

    HILTransceiver *matlab = HIL::getInstance().simulator;

    // registering the HILTransceiver in order to let him know when it has to
    // wait to the control algorithm or not
    // flightPhasesManager->registerToFlightPhase(
    //     AEROBRAKES, bind(&HILTransceiver::setIsAirbrakePhase, matlab, true));

    // flightPhasesManager->registerToFlightPhase(
    //     APOGEE, bind(&HILTransceiver::setIsAirbrakePhase, matlab, false));

    /*-------------- Sensors & Actuators --------------*/
    // Definition of the fake sensors for the simulation
    state.accelerometer = new HILAccelerometer(matlab, N_DATA_ACCEL);
    state.barometer     = new HILBarometer(matlab, N_DATA_BARO);
    state.gps           = new HILGps(matlab, N_DATA_GPS);
    state.gyro          = new HILGyroscope(matlab, N_DATA_GYRO);
    state.magnetometer  = new HILMagnetometer(matlab, N_DATA_MAGN);
    state.kalman        = new HILKalman(matlab, N_DATA_KALM);

    // // Definition of the fake actuators for the simulation
    // HILServo servo(matlab);
    // servo.init();

    /*-------------- [SM] Sensor Manager --------------*/

    // instantiate the sensor manager with the given scheduler
    SensorManager SM({{state.accelerometer, accelConfig},
                      {state.barometer, baroConfig},
                      {state.gps, gpsConfig},
                      {state.gyro, gyroConfig},
                      {state.magnetometer, magnConfig},
                      {state.kalman, kalmConfig}},
                     &scheduler);

    // registering the enabling of the sensors to the flightPhasesManager
    flightPhasesManager->registerToFlightPhase(
        FlightPhases::SIMULATION_STARTED,
        bind(&SensorManager::enableAllSensors, &SM));

    /*---------------- [ADA] ADA ---------------*/
    Main::ADAController &ada_controller = Main::ADAController::getInstance();

    // start the ada controller when the simulation starts
    flightPhasesManager->registerToFlightPhase(
        FlightPhases::SIMULATION_STARTED,
        bind(&Main::ADAController::start, &ada_controller));

    // set reference values when starting calibration
    flightPhasesManager->registerToFlightPhase(
        FlightPhases::CALIBRATION,
        bind(&Main::ADAController::setReferenceValues, &ada_controller,
             Boardcore::ReferenceValues(
                 Main::ADAConfig::DEFAULT_REFERENCE_ALTITUDE,
                 Main::ADAConfig::DEFAULT_REFERENCE_PRESSURE,
                 Main::ADAConfig::DEFAULT_REFERENCE_TEMPERATURE)));

    /*---------------- [NAS] NAS ---------------*/
    Main::NASController &nas_controller = Main::NASController::getInstance();

    // start the ada controller when the simulation starts
    flightPhasesManager->registerToFlightPhase(
        FlightPhases::SIMULATION_STARTED,
        bind(&Main::NASController::start, &nas_controller));

    /*-------------- [CA] Control Algorithm --------------*/
    Main::AirBrakesController &airbrakes_controller =
        Main::AirBrakesController::getInstance();

    airbrakes_controller.setActuatorFunction(setActuator);

    // start the airbrakes controller when the simulation starts
    flightPhasesManager->registerToFlightPhase(
        FlightPhases::SIMULATION_STARTED,
        bind(&Main::AirBrakesController::start, &airbrakes_controller));

    // // definition of the control algorithm
    // MockAirbrakeAlgorithm<HILKalmanData> mockAirbrake(state.kalman, &servo);

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

    /*-------------- Adding tasks to scheduler --------------*/

    // adding the updating of the algorithm to the scheduler
    {
        ActuatorData *actuatorData = HIL::getInstance().getActuatorData();

        /* update AirBrakes 10 Hz*/
        TaskScheduler::function_t update_Airbrake{
            bind(&Main::AirBrakesController::update, &airbrakes_controller)};

        scheduler.addTask(update_Airbrake, (uint32_t)(1000 / CONTROL_FREQ),
                          getNextSchedulerId(&scheduler));

        /* update ADA 50 Hz */
        TaskScheduler::function_t update_ADA{
            bind(&Main::ADAController::update, &ada_controller)};

        scheduler.addTask(update_ADA,
                          (uint32_t)(Main::ADAConfig::UPDATE_PERIOD),
                          getNextSchedulerId(&scheduler));

        TaskScheduler::function_t updateActuatorData_ADA{
            bind(&ActuatorData::addADAState, actuatorData,
                 ada_controller.getAdaState())};

        scheduler.addTask(updateActuatorData_ADA,
                          (uint32_t)(Main::ADAConfig::UPDATE_PERIOD),
                          getNextSchedulerId(&scheduler));

        /* update NAS 50 Hz  */
        TaskScheduler::function_t update_NAS{
            bind(&Main::NASController::update, &nas_controller)};

        scheduler.addTask(update_NAS,
                          (uint32_t)(Main::NASConfig::UPDATE_PERIOD),
                          getNextSchedulerId(&scheduler));

        TaskScheduler::function_t updateActuatorData_NAS{
            bind(&ActuatorData::addNASState, actuatorData,
                 nas_controller.getNasState())};

        scheduler.addTask(updateActuatorData_NAS,
                          (uint32_t)(Main::NASConfig::UPDATE_PERIOD),
                          getNextSchedulerId(&scheduler));
    }

    /*---------- Starting threads --------*/

    HIL::getInstance().start();
    scheduler.start();  // started only the scheduler instead of the SM

    /*---------- Normal execution --------*/

    while (isSimulationRunning)
    {
        Thread::sleep(1000);
    }

    return 0;
}
