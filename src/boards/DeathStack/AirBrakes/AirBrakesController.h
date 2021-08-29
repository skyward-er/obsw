/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#pragma once

#include <AirBrakes/AirBrakesAlgorithm.h>
#include <AirBrakes/AirBrakesData.h>
#include <AirBrakes/AirBrakesServo.h>
#include <System/StackLogger.h>
#include <TimestampTimer.h>
#include <configs/AirBrakesConfig.h>
#include <diagnostic/PrintLogger.h>
#include <events/EventBroker.h>
#include <events/Events.h>
#include <events/FSM.h>
#include <miosix.h>

namespace DeathStackBoard
{

/**
 * @brief AirBrakes state machine
 */
template <class T>
class AirBrakesController : public FSM<AirBrakesController<T>>
{
public:
    AirBrakesController(Sensor<T>& sensor,
                        ServoInterface* servo = new AirBrakesServo());
    ~AirBrakesController();

    // Airbrakes FSM states
    void state_initialization(const Event& ev);
    void state_idle(const Event& ev);
    void state_shadowMode(const Event& ev);
    void state_enabled(const Event& ev);
    void state_end(const Event& ev);
    void state_disabled(const Event& ev);
    void state_testAirbrakes(const Event& ev);

    /**
     * @brief Update the airbrakes control algorithm
     */
    void update();

    void setAirBrakesPosition(float pos);

    float getEstimatedCd() { return algorithm.getEstimatedCd(); }

private:
    /**
     * @brief Incrementally opens the servo with steps of 10°
     */
    void incrementallyOpen();

    /**
     * @brief Incrementally closes the servo with steps of 10°
     */
    void incrementallyClose();

    void logStatus(AirBrakesControllerState state);

    AirBrakesControllerStatus status;
    ServoInterface* servo;
    AirBrakesControlAlgorithm<T> algorithm;
    uint16_t ev_shadow_mode_timeout_id;

    PrintLogger log = Logging::getLogger("deathstack.fsm.arb");
};

template <class T>
AirBrakesController<T>::AirBrakesController(Sensor<T>& sensor,
                                            ServoInterface* servo)
    : FSM<AirBrakesController<T>>(
          &AirBrakesController<T>::state_initialization),
      servo(servo), algorithm(sensor, servo)
{
    memset(&status, 0, sizeof(AirBrakesControllerStatus));
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_ABK);
}

template <class T>
AirBrakesController<T>::~AirBrakesController()
{
    sEventBroker->unsubscribe(this);
}

template <class T>
void AirBrakesController<T>::update()
{
    algorithm.update();
}

template <class T>
void AirBrakesController<T>::state_initialization(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            logStatus(AirBrakesControllerState::INITIALIZATION);

            servo->enable();
            servo->reset();

            this->transition(&AirBrakesController<T>::state_idle);
            break;
        }
        case EV_EXIT:
        {
            break;
        }

        default:
        {
            break;
        }
    }
}

template <class T>
void AirBrakesController<T>::state_idle(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            logStatus(AirBrakesControllerState::IDLE);

            LOG_DEBUG(log, "Eentering state idle");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state idle");
            break;
        }
        case EV_LIFTOFF:
        {
            this->transition(&AirBrakesController<T>::state_shadowMode);
            break;
        }
        case EV_WIGGLE_SERVO:
        {
            servo->selfTest();
            break;
        }
        case EV_RESET_SERVO:
        {
            servo->reset();
            break;
        }
        case EV_TEST_ABK:
        {
            this->transition(&AirBrakesController<T>::state_testAirbrakes);
            break;
        }
        default:
        {
            break;
        }
    }
}

template <class T>
void AirBrakesController<T>::state_shadowMode(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            ev_shadow_mode_timeout_id =
                sEventBroker->postDelayed<SHADOW_MODE_DURATION>(
                    Event{EV_SHADOW_MODE_TIMEOUT}, TOPIC_ABK);

            logStatus(AirBrakesControllerState::SHADOW_MODE);

            LOG_DEBUG(log, "Entering state shadow_mode");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state shadow_mode");
            break;
        }
        case EV_SHADOW_MODE_TIMEOUT:
        {
            this->transition(&AirBrakesController<T>::state_enabled);
            break;
        }
        case EV_DISABLE_ABK:
        {
            this->transition(&AirBrakesController<T>::state_disabled);
            break;
        }
        default:
        {
            break;
        }
    }
}

template <class T>
void AirBrakesController<T>::state_enabled(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            algorithm.begin();

            logStatus(AirBrakesControllerState::ENABLED);

            LOG_DEBUG(log, "Entering state enabled");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state enabled");
            break;
        }
        case EV_APOGEE:
        {
            this->transition(&AirBrakesController<T>::state_end);
            break;
        }
        case EV_DISABLE_ABK:
        {
            this->transition(&AirBrakesController<T>::state_disabled);
            break;
        }
        default:
        {
            break;
        }
    }
}

template <class T>
void AirBrakesController<T>::state_end(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            algorithm.end();
            servo->reset();

            logStatus(AirBrakesControllerState::END);

            LOG_DEBUG(log, "Entering state end");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state end");
            break;
        }

        default:
        {
            break;
        }
    }
}

template <class T>
void AirBrakesController<T>::state_disabled(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            algorithm.end();
            servo->reset();

            logStatus(AirBrakesControllerState::DISABLED);

            LOG_DEBUG(log, "Entering state disabled");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state disabled");
            break;
        }

        default:
        {
            break;
        }
    }
}

template <class T>
void AirBrakesController<T>::state_testAirbrakes(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            LOG_DEBUG(log, "Entering state test_airbrakes");

            incrementallyOpen();
            miosix::Thread::sleep(1000);
            incrementallyClose();
            miosix::Thread::sleep(1000);
            servo->reset();

            logStatus(AirBrakesControllerState::TEST_AEROBRAKES);

            sEventBroker->post(Event{EV_TEST_TIMEOUT}, TOPIC_ABK);
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state test_airbrakes");
            break;
        }
        case EV_TEST_TIMEOUT:
        {
            this->transition(&AirBrakesController<T>::state_idle);
            break;
        }
        default:
        {
            break;
        }
    }
}

template <class T>
void AirBrakesController<T>::incrementallyOpen()
{
    // Equal steps of about 5°
    const int STEPS_NUM        = (servo->MAX_POS - servo->MIN_POS) / 10.0f;
    const float INCREMENT_STEP = (servo->MAX_POS - servo->MIN_POS) / STEPS_NUM;

    float currentStep = servo->MIN_POS;

    for (auto i = 0; i < STEPS_NUM; i++)
    {
        LOG_DEBUG(log, "Servo position : {:.2f}", currentStep);
        servo->set(currentStep);
        currentStep += INCREMENT_STEP;
        miosix::Thread::sleep(1000);
    }

    servo->set(servo->MAX_POS);
}

template <class T>
void AirBrakesController<T>::incrementallyClose()
{
    // Equal steps of about 5°
    const int STEPS_NUM        = (servo->MAX_POS - servo->MIN_POS) / 10.0f;
    const float INCREMENT_STEP = (servo->MAX_POS - servo->MIN_POS) / STEPS_NUM;

    float currentStep = servo->MAX_POS;

    for (auto i = 0; i < STEPS_NUM; i++)
    {
        LOG_DEBUG(log, "Servo position : {:.2f}", currentStep);
        servo->set(currentStep);
        currentStep -= INCREMENT_STEP;
        miosix::Thread::sleep(1000);
    }

    servo->set(servo->MIN_POS);
}

template <class T>
void AirBrakesController<T>::logStatus(AirBrakesControllerState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    LoggerService::getInstance()->log(status);

    StackLogger::getInstance()->updateStack(THID_ABK_FSM);
}

template <class T>
void AirBrakesController<T>::setAirBrakesPosition(float pos)
{
    servo->set(pos);
}

}  // namespace DeathStackBoard