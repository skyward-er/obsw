#pragma once

#include <Motor/StateMachines/MEAController/MEAController.h>
#include <Motor/StateMachines/MEAController/MEAControllerData.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

using namespace Boardcore;
using namespace Common;
using namespace miosix;

namespace Motor
{

MEAController::MEAController()
    : FSM{&MEAController::state_init, STACK_DEFAULT_FOR_PTHREAD,
          BoardScheduler::meaControllerPriority()} // config file? 
// Add MEA obv;
{
    EventBroker::getInstance().subscribe(this, TOPIC_MEA);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
};

MEAControllerState MEAController::getMEAControllerState() { return state; }

bool MEAController::start()
{
    // need to update Motor BoardScheduler?
    TaskScheduler& scheduler = getModule<BoardScheduler>()->getMeaScheduler();

    size_t result = scheduler.addTask(
        [this]() { update() }, /* Need to create config file for scheduler */);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to start MEAController");
        return false;
    }

    if (!FSM::start())
    {
        LOG_ERR(logger, "Failed to start MEA FSM");
        return false;
    }

    return true;
}

void MEAController::updateAndLogStatus(MEAControllerState state)
{
    this->state                = state;
    MEAControllerStatus status = {TimestampTimer::getTimestamp(), state};
    sdLogger.log(status);
}

void MEAController::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(MEAControllerState::INIT);
            transition(&MEAController::state_ready);  // check this transition
            break;
        }
    }
}

void MEAController::state_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(MEAControllerState::ACTIVE);
            break;
        }

        case FLIGHT_ARMED:
        case MEA_FORCE_START:
        {
            transition(&MEAController::state_active);
            break;
        }
    }
}

void MEAController::state_active(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(MEAControllerState::ACTIVE);
            break;
        }

        case FLIGHT_DISARMED:
        case MEA_FORCE_STOP:
        {
            transition(&MEAController::state_ready);
            break;
        }

        case FLIGHT_LANDING_DETECTED:
        case FLIGHT_MOTOR_SHUTDOWN:
        case FLIGHT_APOGEE_DETECTED:
        case FLIGHT_DPL_ALT_DETECTED:
        {
            transition(&MEAController::state_end);
            break;
        }
    }
}

void MEAController::state_end(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(MEAControllerState::END);
            break;
        }
    }
}

}  // namespace Motor
