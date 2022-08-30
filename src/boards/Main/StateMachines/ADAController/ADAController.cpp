/* Copyright (c) 2022 Skyward Experimental Rocketry
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "ADAController.h"

#include <Main/BoardScheduler.h>
#include <Main/Configs/AirBrakesControllerConfig.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/AirBrakesController/AirBrakesController.h>
#include <common/events/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;
using namespace Main::ADAConfig;
using namespace Main::AirBrakesControllerConfigs;
using namespace Common;

namespace Main
{

bool ADAController::start()
{
    BoardScheduler::getInstance().getScheduler().addTask(
        std::bind(&ADAController::update, this), ADAConfig::UPDATE_PERIOD,
        TaskScheduler::Policy::RECOVER);

    return ActiveObject::start();
}

void ADAController::update()
{
    Boardcore::MS5803Data barometerData =
        Sensors::getInstance().getMS5803LastSample();

    switch (status.state)
    {
        case ADAControllerState::CALIBRATING:
        {
            // TODO: Implement calibration

            EventBroker::getInstance().post(ADA_READY, TOPIC_ADA);

            break;
        }
        case ADAControllerState::SHADOW_MODE:
        {
            // During shadow mode DO NOT send events

            ada.update(barometerData.pressure);

            // Check for apogee
            if (ada.getState().verticalSpeed < APOGEE_VERTICAL_SPEED_TARGET)
            {
                detectedApogeeEvents++;

                if (detectedApogeeEvents > APOGEE_N_SAMPLES)
                {
                    // Apogee detected in shadow mode!
                    Logger::getInstance().log(ApogeeEvent{
                        TimestampTimer::getTimestamp(), status.state});
                }
            }
            else
            {
                detectedApogeeEvents = 0;
            }

            break;
        }
        case ADAControllerState::ACTIVE:
        {
            ada.update(barometerData.pressure);

            // Check for apogee
            if (ada.getState().verticalSpeed < APOGEE_VERTICAL_SPEED_TARGET)
            {
                detectedApogeeEvents++;

                if (detectedApogeeEvents > APOGEE_N_SAMPLES)
                {
                    // Apogee detected
                    EventBroker::getInstance().post(FLIGHT_APOGEE_DETECTED,
                                                    TOPIC_FLIGHT);
                    Logger::getInstance().log(ApogeeEvent{
                        TimestampTimer::getTimestamp(), status.state});
                }
            }
            else
            {
                detectedApogeeEvents = 0;
            }

            // Check if the airbrakes need to be disabled. This will happen
            // before the apogee so that the airbrakes are retracted to prevent
            // the parachutes cords from entangling.
            if (ada.getState().verticalSpeed < DISABLE_VERTICAL_SPEED_TARGET)
            {
                detectedAbkDisableEvents++;

                if (detectedAbkDisableEvents > ABK_DISABLE_N_SAMPLES &&
                    AirBrakesController::getInstance().getStatus().state !=
                        AirBrakesControllerState::END)
                    EventBroker::getInstance().post(ABK_DISABLE, TOPIC_ABK);
            }
            else
            {
                detectedAbkDisableEvents = 0;
            }

            break;
        }
        case ADAControllerState::PRESSURE_STABILIZATION:
        {
            // During pressure stabilization DO NOT send event

            ada.update(barometerData.pressure);

            if (ada.getState().aglAltitude <= deploymentAltitude)
            {
                detectedDeploymentEvents++;

                if (detectedDeploymentEvents >= DEPLOYMENT_N_SAMPLES)
                {
                    // Deployment event detected during pressure stabilization!
                    Logger::getInstance().log(DeploymentEvent{
                        TimestampTimer::getTimestamp(), status.state});
                }
            }
            else
            {
                detectedDeploymentEvents = 0;
            }

            break;
        }
        case ADAControllerState::DROGUE_DESCENT:
        {
            ada.update(barometerData.pressure);

            if (ada.getState().aglAltitude <= deploymentAltitude)
            {
                detectedDeploymentEvents++;

                if (detectedDeploymentEvents >= DEPLOYMENT_N_SAMPLES)
                {
                    // Deployment event detected
                    EventBroker::getInstance().post(FLIGHT_DPL_ALT_DETECTED,
                                                    TOPIC_FLIGHT);
                    Logger::getInstance().log(DeploymentEvent{
                        TimestampTimer::getTimestamp(), status.state});
                }
            }
            else
            {
                detectedDeploymentEvents = 0;
            }

            break;
        }
        case ADAControllerState::TERMINAL_DESCENT:
        {
            ada.update(barometerData.pressure);

            if (abs(ada.getState().verticalSpeed) <
                LANDING_VERTICAL_SPEED_MAG_TARGET)
            {
                detectedLandingEvents++;

                if (detectedLandingEvents > LANDING_N_SAMPLES)
                {
                    // Landing detected
                    EventBroker::getInstance().post(FLIGHT_LANDING_DETECTED,
                                                    TOPIC_FLIGHT);
                }
            }
        }

        case ADAControllerState::UNINIT:
        case ADAControllerState::IDLE:
        case ADAControllerState::READY:
        case ADAControllerState::LANDED:
        {
        }
    }

    Logger::getInstance().log(ada.getState());

#ifdef HILSimulation
    // useful only for hil testing
    updateData(ada.getState());
#endif  // HILSimulation
}

ADAControllerStatus ADAController::getStatus()
{
    PauseKernelLock lock;
    return status;
}

ADAState ADAController::getAdaState()
{
    PauseKernelLock lock;
    return ada.getState();
}

void ADAController::setDeploymentAltitude(float altitude)
{
    deploymentAltitude = altitude;
}

void ADAController::setReferenceAltitude(float altitude)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock l;

    ReferenceValues reference = ada.getReferenceValues();
    reference.altitude        = altitude;
    ada.setReferenceValues(reference);
}

void ADAController::setReferenceTemperature(float temperature)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock l;

    ReferenceValues reference = ada.getReferenceValues();
    reference.temperature     = temperature;
    ada.setReferenceValues(reference);
}

void ADAController::setReferenceValues(const ReferenceValues reference)
{
    ada.setReferenceValues(reference);
}

ReferenceValues ADAController::getReferenceValues()
{
    return ada.getReferenceValues();
}

void ADAController::state_idle(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(ADAControllerState::IDLE);
        }
        case ADA_CALIBRATE:
        {
            return transition(&ADAController::state_calibrating);
        }
        case FLIGHT_MISSION_TIMEOUT:
        {
            return transition(&ADAController::state_landed);
        }
    }
}

void ADAController::state_calibrating(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(ADAControllerState::CALIBRATING);

            return calibrate();
        }
        case ADA_READY:
        {
            return transition(&ADAController::state_ready);
        }
        case FLIGHT_MISSION_TIMEOUT:
        {
            return transition(&ADAController::state_landed);
        }
    }
}

void ADAController::state_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(ADAControllerState::READY);
        }
        case ADA_CALIBRATE:
        {
            return transition(&ADAController::state_calibrating);
        }
        case FLIGHT_LIFTOFF:
        {
            return transition(&ADAController::state_shadow_mode);
        }
        case FLIGHT_MISSION_TIMEOUT:
        {
            return transition(&ADAController::state_landed);
        }
    }
}

void ADAController::state_shadow_mode(const Event& event)
{
    static uint16_t shadowModeTimeoutEventId = -1;

    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(ADAControllerState::SHADOW_MODE);

            shadowModeTimeoutEventId =
                EventBroker::getInstance()
                    .postDelayed<ADAConfig::SHADOW_MODE_TIMEOUT>(
                        Boardcore::Event{ADA_SHADOW_MODE_TIMEOUT}, TOPIC_ADA);
            break;
        }
        case ADA_SHADOW_MODE_TIMEOUT:
        {
            return transition(&ADAController::state_active);
        }
        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(shadowModeTimeoutEventId);
            break;
        }
        case FLIGHT_MISSION_TIMEOUT:
        {
            return transition(&ADAController::state_landed);
        }
    }
}

void ADAController::state_active(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(ADAControllerState::ACTIVE);
        }
        case FLIGHT_APOGEE_DETECTED:
        {
            return transition(&ADAController::state_pressure_stabilization);
        }
        case FLIGHT_MISSION_TIMEOUT:
        {
            return transition(&ADAController::state_landed);
        }
    }
}

void ADAController::state_pressure_stabilization(const Event& event)
{
    static uint16_t pressStabTimeoutEventId = -1;

    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(ADAControllerState::PRESSURE_STABILIZATION);

            pressStabTimeoutEventId =
                EventBroker::getInstance().postDelayed<PRES_STAB_TIMEOUT>(
                    Boardcore::Event{ADA_PRESS_STAB_TIMEOUT}, TOPIC_ADA);
            break;
        }
        case ADA_PRESS_STAB_TIMEOUT:
        {
            return transition(&ADAController::state_drogue_descent);
        }
        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(pressStabTimeoutEventId);
            break;
        }
        case FLIGHT_MISSION_TIMEOUT:
        {
            return transition(&ADAController::state_landed);
        }
    }
}

void ADAController::state_drogue_descent(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(ADAControllerState::DROGUE_DESCENT);
        }
        case FLIGHT_DPL_ALT_DETECTED:
        {
            return transition(&ADAController::state_terminal_descent);
        }
        case FLIGHT_MISSION_TIMEOUT:
        {
            return transition(&ADAController::state_landed);
        }
    }
}

void ADAController::state_terminal_descent(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(ADAControllerState::TERMINAL_DESCENT);
        }
        case FLIGHT_LANDING_DETECTED:
        {
            return transition(&ADAController::state_landed);
        }
        case FLIGHT_MISSION_TIMEOUT:
        {
            return transition(&ADAController::state_landed);
        }
    }
}

void ADAController::state_landed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(ADAControllerState::LANDED);
        }
    }
}

void ADAController::setUpdateDataFunction(
    std::function<void(Boardcore::ADAState)> updateData)
{
    this->updateData = updateData;
}

ADAController::ADAController()
    : FSM(&ADAController::state_idle),
      ada({DEFAULT_REFERENCE_ALTITUDE, DEFAULT_REFERENCE_PRESSURE,
           DEFAULT_REFERENCE_TEMPERATURE},
          getADAKalmanConfig()),
      updateData([](Boardcore::ADAState) {})
{
    EventBroker::getInstance().subscribe(this, TOPIC_ADA);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

ADAController::~ADAController()
{
    EventBroker::getInstance().unsubscribe(this);
}

void ADAController::logStatus(ADAControllerState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    Logger::getInstance().log(status);
}

ADA::KalmanFilter::KalmanConfig ADAController::getADAKalmanConfig()
{
    ADA::KalmanFilter::MatrixNN F_INIT;
    // clang-format off
    F_INIT <<
        1.0, SAMPLING_PERIOD, 0.5f * SAMPLING_PERIOD * SAMPLING_PERIOD,
        0.0, 1.0,             SAMPLING_PERIOD,
        // cppcheck-suppress constStatement
        0.0, 0.0,             1.0;
    // clang-format on
    ADA::KalmanFilter::MatrixPN H_INIT{1.0, 0.0, 0.0};
    ADA::KalmanFilter::MatrixNN P_INIT;
    // cppcheck-suppress constStatement
    P_INIT << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    ADA::KalmanFilter::MatrixNN Q_INIT;
    // cppcheck-suppress constStatement
    Q_INIT << 30.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 2.5f;
    ADA::KalmanFilter::MatrixPP R_INIT{4000.0f};

    return {F_INIT,
            H_INIT,
            Q_INIT,
            R_INIT,
            P_INIT,
            ADA::KalmanFilter::CVectorN(DEFAULT_REFERENCE_PRESSURE, 0,
                                        KALMAN_INITIAL_ACCELERATION)};
}

void ADAController::calibrate()
{
    // ...
}

}  // namespace Main
