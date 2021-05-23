#include "StateMachines.h"

#include "ADA/ADAController.h"
#include "AeroBrakesController/AeroBrakesController.h"
#include "DeploymentController/DeploymentController.h"
#include "FlightModeManager/FlightModeManager.h"
#include "NavigationSystem/NASController.h"

namespace DeathStackBoard
{
StateMachines::StateMachines(BMX160& imu, MS580301BA07& press, UbloxGPS& gps,
                             TaskScheduler* scheduler)
{
    ada_controller = new ADAControllerType(press, gps);
    dpl_controller = new DeploymentController();
    nas_controller = new NASControllerType(imu, press, gps);
    arb_controller = new AeroBrakesControllerType(nas_controller->getNAS());
    fmm            = new FlightModeManager();

    addAlgorithmsToScheduler(scheduler);
}

StateMachines::~StateMachines()
{
    delete arb_controller;
    delete nas_controller;
    delete ada_controller;
    delete dpl_controller;
    delete fmm;
}

void StateMachines::start()
{
    fmm->start();
    dpl_controller->start();
    ada_controller->start();
    nas_controller->start();
    arb_controller->start();
}

void StateMachines::addAlgorithmsToScheduler(TaskScheduler* scheduler)
{
    uint64_t start_time = miosix::getTick() + 10;

    scheduler->add(std::bind(&ADAControllerType::update, ada_controller),
                   ADA_UPDATE_PERIOD, scheduler->getTaskStats().back().id + 1,
                   start_time);

    scheduler->add(std::bind(&NASControllerType::update, nas_controller),
                   NAS_UPDATE_PERIOD, scheduler->getTaskStats().back().id + 1,
                   start_time);

    scheduler->add(std::bind(&AeroBrakesControllerType::update, arb_controller),
                   ABK_UPDATE_PERIOD, scheduler->getTaskStats().back().id + 1,
                   start_time);
}

}  // namespace DeathStackBoard