/*
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alvise de' Faveri Tron
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdexcept>

#include "DeathStack/events/Events.h"
#include "DeathStack/configs/DeploymentConfig.h"
#include "DeploymentController.h"

#include "Motor/MotorDriver.h"
#include "events/EventBroker.h"

namespace DeathStackBoard
{

DeploymentController::DeploymentController()
    : HSM(&DeploymentController::state_initialization, 4096, 2), motor(),
      servo_rk(TIM4_DATA), servo_l(TIM8_DATA)
{
    // No conflicts on TIM4, enable PWM immediately
    configureTIM4Servos();

    memset(&status, 0, sizeof(DeploymentStatus));

    sEventBroker->subscribe(this, TOPIC_DEPLOYMENT);
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_TC);
}

DeploymentController::~DeploymentController()
{
    sEventBroker->unsubscribe(this);
}

State DeploymentController::state_initialization(const Event& ev)
{
    // Nothing to do during initialization

    UNUSED(ev);
    return transition(&DeploymentController::state_idle);
}

State DeploymentController::state_idle(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            logStatus(DeploymentCTRLState::DPL_IDLE);
            // Process deferred events
            try
            {
                while (!deferred_events.isEmpty())
                {
                    postEvent(deferred_events.pop());
                }
            }
            catch (...)
            {
                TRACE("Tried to pop empty circularbuffer!\n");
            }

            TRACE("[DPL_CTRL] state_idle ENTRY\n");
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            TRACE("[DPL_CTRL] state_idle EXIT\n");

            break;
        }
        case EV_NC_OPEN:
        {
            retState = transition(&DeploymentController::state_openingNosecone);
            break;
        }
        case EV_CUT_MAIN:
        {
            retState = transition(&DeploymentController::state_cuttingMain);
            break;
        }
        case EV_CUT_DROGUE:
        {
            retState = transition(&DeploymentController::state_cuttingDrogue);
            break;
        }
        case EV_DPL_ALTITUDE:
        {
            // TIM8 Pins are conflicting with the motor driver pins, don't
            // configure them until necessary
            configureTIM8Servos();
            break;
        }
        case EV_START_ROGALLO_CONTROL:
        {
            TRACE("Controlling rogallo\n");
            controlRogallo();
            break;
        }
        default:
        {
            retState = tran_super(&DeploymentController::Hsm_top);
            break;
        }
    }
    return retState;
}

State DeploymentController::state_openingNosecone(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            // Start the motor to open the nosecone
            motor.start(MOTOR_OPEN_DIR);
            logStatus(DeploymentCTRLState::OPENING_NC);

            ev_open_timeout_id = sEventBroker->postDelayed(
                Event{EV_TIMEOUT_MOT_OPEN}, TOPIC_DEPLOYMENT, NC_OPEN_TIMEOUT);

            TRACE("[DPL_CTRL] state_openingNosecone ENTRY\n");
            break;
        }
        case EV_INIT:
        {
            TRACE("[DPL_CTRL] state_openingNosecone INIT\n");

            retState = transition(&DeploymentController::state_spinning);

            break;
        }
        case EV_EXIT:
        {
            // Stop the motor
            motor.stop();

            sEventBroker->removeDelayed(ev_min_open_time_id);
            sEventBroker->removeDelayed(ev_open_timeout_id);

            TRACE("[DPL_CTRL] state_openingNosecone EXIT\n");

            break;
        }
        case EV_TIMEOUT_MOT_OPEN:
        {
            retState = transition(&DeploymentController::state_idle);
            break;
        }
        case EV_CUT_DROGUE:
        {
            deferred_events.put(ev);
            break;
        }
        case EV_CUT_MAIN:
        {
            deferred_events.put(ev);
            break;
        }
        default:
        {
            retState = tran_super(&DeploymentController::Hsm_top);
            break;
        }
    }
    return retState;
}

State DeploymentController::state_spinning(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            ev_min_open_time_id = sEventBroker->postDelayed(
                Event{EV_MOT_MIN_OPEN_TIME}, TOPIC_DEPLOYMENT,
                NC_MINIMUM_OPENING_TIME);

            TRACE("[DPL_CTRL] state_spinning ENTRY\n");
            logStatus(DeploymentCTRLState::SPINNING);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            TRACE("[DPL_CTRL] state_spinning EXIT\n");

            break;
        }
        case EV_NC_DETACHED:
        {
            retState =
                transition(&DeploymentController::state_awaitingOpenTime);
            break;
        }
        case EV_MOT_MIN_OPEN_TIME:
        {
            retState =
                transition(&DeploymentController::state_awaitingDetachment);
            break;
        }
        default:
        {
            retState = tran_super(&DeploymentController::state_openingNosecone);
            break;
        }
    }
    return retState;
}

State DeploymentController::state_awaitingOpenTime(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("[DPL_CTRL] state_awaitingOpenTime ENTRY\n");
            logStatus(DeploymentCTRLState::AWAITING_MINOPENTIME);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            TRACE("[DPL_CTRL] state_awaitingOpenTime EXIT\n");

            break;
        }
        case EV_MOT_MIN_OPEN_TIME:
        {
            retState = transition(&DeploymentController::state_idle);
            break;
        }
        default:
        {
            retState = tran_super(&DeploymentController::state_openingNosecone);
            break;
        }
    }
    return retState;
}

State DeploymentController::state_awaitingDetachment(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("[DPL_CTRL] state_awaitingDetachment ENTRY\n");
            logStatus(DeploymentCTRLState::AWAITING_DETACHMENT);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            TRACE("[DPL_CTRL] state_awaitingOpenTime EXIT\n");

            break;
        }
        case EV_NC_DETACHED:
        {
            retState = transition(&DeploymentController::state_idle);
            break;
        }
        default:
        {
            retState = tran_super(&DeploymentController::state_openingNosecone);
            break;
        }
    }
    return retState;
}

State DeploymentController::state_cuttingDrogue(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            // Cutting drogue
            cutter.startCutDrogue();
            logStatus(DeploymentCTRLState::CUTTING_DROGUE);

            ev_cut_timeout_id = sEventBroker->postDelayed(
                {EV_TIMEOUT_CUTTING}, TOPIC_DEPLOYMENT,
                MAXIMUM_CUTTING_DURATION);

            TRACE("[DPL_CTRL] state_cuttingDrogue ENTRY\n");
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            cutter.stopCutDrogue();

            sEventBroker->removeDelayed(ev_cut_timeout_id);
            TRACE("[DPL_CTRL] state_cuttingDrogue EXIT\n");

            break;
        }
        case EV_TIMEOUT_CUTTING:
        {
            retState = transition(&DeploymentController::state_idle);
            break;
        }
        case EV_CUT_MAIN:
        {
            deferred_events.put(ev);
            break;
        }
        default:
        {
            retState = tran_super(&DeploymentController::Hsm_top);
            break;
        }
    }
    return retState;
}

State DeploymentController::state_cuttingMain(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            // Cutting rogallina
            cutter.startCutMainChute();
            logStatus(DeploymentCTRLState::CUTTING_MAIN);

            ev_cut_timeout_id = sEventBroker->postDelayed(
                {EV_TIMEOUT_CUTTING}, TOPIC_DEPLOYMENT,
                MAXIMUM_CUTTING_DURATION);

            TRACE("[DPL_CTRL] state_cuttingMain ENTRY\n");
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            cutter.stopCutMainChute();

            sEventBroker->removeDelayed(ev_cut_timeout_id);

            TRACE("[DPL_CTRL] state_cuttingMain EXIT\n");

            break;
        }
        case EV_CUT_DROGUE:
        {
            deferred_events.put(ev);
            break;
        }
        case EV_TIMEOUT_CUTTING:
        {
            retState = transition(&DeploymentController::state_idle);
            break;
        }
        default:
        {
            retState = tran_super(&DeploymentController::Hsm_top);
            break;
        }
    }
    return retState;
}

void DeploymentController::configureTIM4Servos()
{
    {
        miosix::FastInterruptDisableLock dLock;
        miosix::nosecone::rogP1::mode(miosix::Mode::ALTERNATE);
        miosix::nosecone::rogP2::mode(miosix::Mode::ALTERNATE);

        miosix::nosecone::rogP1::alternateFunction(2);
        miosix::nosecone::rogP2::alternateFunction(2);
    }
    servo_rk.enable(SERVO_RIGHT_CH);
    servo_rk.enable(SERVO_KEEL_CH);

    servo_rk.setPosition(SERVO_RIGHT_CH, SERVO_R_RESET_POS);
    servo_rk.setPosition(SERVO_KEEL_CH, SERVO_K_RESET_POS);

    servo_rk.start();
}

// The pins connected to TIM8 are shared with the nosecone driver. Don't
// enable the timer until we have to drive the servos
void DeploymentController::configureTIM8Servos()
{
    {
        miosix::FastInterruptDisableLock dLock;
        miosix::nosecone::motP1::mode(miosix::Mode::ALTERNATE);
        miosix::nosecone::motP1::alternateFunction(3);
    }
    servo_l.enable(SERVO_LEFT_CH);
    servo_l.setPosition(SERVO_LEFT_CH, SERVO_L_RESET_POS);

    servo_l.start();
}

void DeploymentController::controlRogallo()
{
    // Ensure TIM8 is correctly configured
    configureTIM8Servos();

    // Wait for TIM8 servo to go to position
    Thread::sleep(2000);

    // Drive the left servo
    servo_l.setPosition(SERVO_LEFT_CH, SERVO_L_CONTROL_POS);
    // Wait then reset position
    Thread::sleep(4000);
    servo_l.setPosition(SERVO_LEFT_CH, SERVO_L_RESET_POS);
    Thread::sleep(4000);

    // Drive the keel servo
    servo_rk.setPosition(SERVO_KEEL_CH, SERVO_K_CONTROL_POS);
    // Wait then reset position
    Thread::sleep(4000);
    servo_rk.setPosition(SERVO_KEEL_CH, SERVO_K_RESET_POS);
    Thread::sleep(4000);

    // Drive the keel servo
    servo_rk.setPosition(SERVO_RIGHT_CH, SERVO_R_CONTROL_POS);
    // Wait then reset position
    Thread::sleep(4000);
    servo_rk.setPosition(SERVO_RIGHT_CH, SERVO_R_RESET_POS);
    Thread::sleep(4000);
}

}  // namespace DeathStackBoard
