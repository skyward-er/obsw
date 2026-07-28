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
          BoardScheduler::meaControllerPriority()},
      mea{}
{
    EventBroker::getInstance().subscribe(this, TOPIC_MEA);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
};

MEAControllerState MEAController::getMEAControllerState() { return state.load(); }

Boardcore::MEAState MEAController::getMEAState()
{
    Lock<FastMutex> lock{meaMutex};

    auto rawData = mea.getMEA_Out();

    uint64_t timestamp = TimestampTimer::getTimestamp();

    MEAState state(timestamp, rawData.Mass);

    return state;
}

bool MEAController::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->mea();

    size_t result =
        scheduler.addTask([this]() { update(); },
                          getModule<BoardScheduler>()->meaControllerPriority());

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

void MEAController::update()
{
    Lock<FastMutex> lock{meaMutex};

    Sensors* sensors = getModule<Sensors>();
    auto firingHSM   = getModule<FiringSequenceHSM>();

    float CCPTMeasure  = sensors->getMainCCPressure().pressure;
    uint64_t timestamp = TimestampTimer::getTimestamp();
    float mainPosition = sensors->getMainFuelPosition().position;
    uint8_t hsmState   = static_cast<uint8_t>(firingHSM->getState());

    MEA::MEAIn in = {CCPTMeasure, timestamp, mainPosition, hsmState};

    mea.setMEA_In(in);
    mea.step();

    timestamp = TimestampTimer::getTimestamp();
    sdLogger.log(MEALogsWrapper{timestamp, mea.getMEA_Logs_OBSW()});
}

void MEAController::calibrate() { mea.initialize(); }

void MEAController::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(MEAControllerState::INIT);
            break;
        }

        case MEA_CALIBRATE:
        {
            transition(&MEAController::state_calibrate);
            break;
        }
    }
}

void MEAController::state_calibrate(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(MEAControllerState::CALIBRATING);
            calibrate();
            EventBroker::getInstance().post(MEA_READY, TOPIC_MEA);
            break;
        }
        case MEA_READY:
        {
            transition(&MEAController::state_ready);
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

        case MEA_RESET:
        {
            [[fallthrough]];
        }

        case MEA_CALIBRATE:
        {
            transition(&MEAController::state_calibrate);
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
