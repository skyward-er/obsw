#include "StateMachines.h"

#include "DeploymentController/DeploymentController.h"
#include "FlightModeManager/FlightModeManager.h"
#include "ADA/ADAController.h"
#include "NavigationSystem/NASController.h"
#include "AeroBrakesController/AeroBrakesController.h"

namespace DeathStackBoard
{
StateMachines::StateMachines(BMX160& imu, MS580301BA07& press, UbloxGPS& gps)
{
    ada_controller = new ADAControllerType(press, gps);
    dpl_controller = new DeploymentController();
    nas_controller = new NASControllerType(imu, press, gps);
    arb_controller = new AeroBrakesControllerType(nas_controller->getNAS());
    fmm            = new FlightModeManager();
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

}  // namespace DeathStackBoard