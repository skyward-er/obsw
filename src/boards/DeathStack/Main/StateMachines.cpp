/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <AirBrakes/AirBrakesController.h>
#include <ApogeeDetectionAlgorithm/ADAController.h>
#include <Deployment/DeploymentController.h>
#include <FlightModeManager/FMMController.h>
#include <FlightStatsRecorder/FSRController.h>
#include <Main/StateMachines.h>
#include <NavigationAttitudeSystem/NASController.h>
#include <System/TaskID.h>

#ifdef HARDWARE_IN_THE_LOOP
#include <hardware_in_the_loop/HIL.h>
#endif

namespace DeathStackBoard
{
StateMachines::StateMachines(IMUType& imu, PressType& press, GPSType& gps,
                             TaskScheduler* scheduler)
{
    ada_controller = new ADAControllerType(press, gps);
    dpl_controller = new DeploymentController();
    nas_controller = new NASControllerType(imu, press, gps);
    arb_controller = new AirBrakesControllerType(nas_controller->getNAS());
    fmm            = new FMMController();

#ifdef HARDWARE_IN_THE_LOOP
    HIL::getInstance()->setNAS(&nas_controller->getNAS());
#endif

    addAlgorithmsToScheduler(scheduler);
}

StateMachines::~StateMachines()
{
    delete ada_controller;
    delete dpl_controller;
    delete nas_controller;
    delete arb_controller;
    delete fmm;
}

bool StateMachines::start()
{
    return fmm->start() && dpl_controller->start() && ada_controller->start() &&
           nas_controller->start() && arb_controller->start();
}

void StateMachines::addAlgorithmsToScheduler(TaskScheduler* scheduler)
{
    uint64_t start_time = miosix::getTick() + 10;

    scheduler->add(std::bind(&ADAControllerType::update, ada_controller),
                   ADA_UPDATE_PERIOD, TASK_ADA_ID, start_time);

    scheduler->add(std::bind(&NASControllerType::update, nas_controller),
                   NAS_UPDATE_PERIOD, TASK_NAS_ID, start_time);

    scheduler->add(std::bind(&AirBrakesControllerType::update, arb_controller),
                   ABK_UPDATE_PERIOD, TASK_ABK_ID, start_time);
}

void StateMachines::setReferenceTemperature(float t)
{
    ada_controller->setReferenceTemperature(t);
    nas_controller->setReferenceTemperature(t);
}

void StateMachines::setInitialOrientation(float roll, float pitch, float yaw)
{
    nas_controller->setInitialOrientation(roll, pitch, yaw);
}

void StateMachines::setReferenceAltitude(float altitude)
{
    ada_controller->setReferenceAltitude(altitude);
    nas_controller->setReferenceAltitude(altitude);
}

void StateMachines::setInitialCoordinates(float latitude, float longitude)
{
    nas_controller->setInitialCoordinates(latitude, longitude);
}

}  // namespace DeathStackBoard