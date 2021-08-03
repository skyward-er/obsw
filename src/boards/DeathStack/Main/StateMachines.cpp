#include "StateMachines.h"

#include "ADA/ADAController.h"
#include "AeroBrakesController/AeroBrakesController.h"
#include "DeploymentController/DeploymentController.h"
#include "FlightModeManager/FlightModeManager.h"
#include "FlightStatsRecorder/FlightStatsRecorder.h"
#include "NavigationSystem/NASController.h"

#include "System/TaskID.h"

#ifdef HARDWARE_IN_THE_LOOP
#include "hardware_in_the_loop/HIL.h"
#endif

namespace DeathStackBoard
{
StateMachines::StateMachines(IMUType& imu, PressType& press, GPSType& gps,
                             TaskScheduler* scheduler)
{
    ada_controller = new ADAControllerType(press, gps);
    dpl_controller = new DeploymentController();
    nas_controller = new NASControllerType(imu, press, gps);
    arb_controller = new AeroBrakesControllerType(nas_controller->getNAS());
    fmm            = new FlightModeManager();

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

    scheduler->add(std::bind(&AeroBrakesControllerType::update, arb_controller),
                   ABK_UPDATE_PERIOD, TASK_ABK_ID, start_time);

    // add lambda to log scheduler tasks statistics
    scheduler->add(
        [&]() {
            /*std::vector<TaskStatResult> scheduler_stats =
                scheduler->getTaskStats();

            for (TaskStatResult stat : scheduler_stats)
            {
                logger.log(stat);
            }*/

            StackLogger::getInstance()->updateStack(THID_TASK_SCHEDULER);
        },
        1000,  // 1 hz
        TASK_SCHEDULER_STATS_ID, start_time);
}

void StateMachines::setInitialOrientation(float roll, float pitch, float yaw)
{
    nas_controller->setInitialOrientation(roll, pitch, yaw);
}

}  // namespace DeathStackBoard