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

#include <Common.h>
#include <events/EventBroker.h>

#include "TimestampTimer.h"
#include "miosix.h"
#include "scheduler/TaskScheduler.h"
#include "sensors/SensorManager.h"

/* HIL includes */
#include "hardware_in_the_loop/HIL.h"

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
    flightPhasesManager->registerToFlightPhase(
        SIMULATION_STARTED, bind(&SensorManager::enableAllSensors, &SM));

    /*-------------- [CA] Control Algorithm --------------*/

    // definition of the control algorithm
    MockAerobrakeAlgorithm<HILKalmanData> mockAerobrake(state.kalman, &servo);

    // registering the starting and ending of the algorithm in base of the phase
    flightPhasesManager->registerToFlightPhase(
        AEROBRAKES,
        bind(&MockAerobrakeAlgorithm<HILKalmanData>::begin, &mockAerobrake));

    flightPhasesManager->registerToFlightPhase(
        APOGEE,
        bind(&MockAerobrakeAlgorithm<HILKalmanData>::end, &mockAerobrake));

    /*-------------- Adding tasks to scheduler --------------*/

    // adding the updating of the algorithm to the scheduler
    {
        TaskScheduler::function_t update_Aerobrake{bind(
            &MockAerobrakeAlgorithm<HILKalmanData>::update, &mockAerobrake)};

        scheduler.add(update_Aerobrake, (uint32_t)(1000 / CONTROL_FREQ),
                      getNextSchedulerId(&scheduler));
    }

    /*---------- Starting threads --------*/

    matlab->start();
    scheduler.start();  // started only the scheduler instead of the SM

    /*---------- Normal execution --------*/

    while (isSimulationRunning)
    {
        Thread::sleep(1000);
    }

    return 0;
}
