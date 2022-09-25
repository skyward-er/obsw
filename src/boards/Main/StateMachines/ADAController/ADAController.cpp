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
#include <Main/Configs/NASConfig.h>
#include <Main/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/AirBrakesController/AirBrakesController.h>
#include <common/ReferenceConfig.h>
#include <common/events/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;
using namespace Main::ADAConfig;
using namespace Main::AirBrakesControllerConfig;
using namespace Main::NASConfig;
using namespace Common;
using namespace Common::ReferenceConfig;

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
    auto barometerData = Sensors::getInstance().getStaticPressureLastSample();

    switch (status.state)
    {
        case ADAControllerState::ARMED:
        {
            ada.update(barometerData.pressure);
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
                    Logger::getInstance().log(
                        ApogeeEvent{TimestampTimer::getTimestamp()});
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
                    Logger::getInstance().log(
                        ApogeeEvent{TimestampTimer::getTimestamp()});
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
                    Logger::getInstance().log(
                        MainEvent{TimestampTimer::getTimestamp()});
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
                    Logger::getInstance().log(
                        MainEvent{TimestampTimer::getTimestamp()});
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
                    Logger::getInstance().log(
                        LandingEvent{TimestampTimer::getTimestamp()});
                }
            }
        }

        case ADAControllerState::UNINIT:
        case ADAControllerState::IDLE:
        case ADAControllerState::READY:
        case ADAControllerState::CALIBRATING:
        case ADAControllerState::LANDED:
        {
        }
    }

    Logger::getInstance().log(ada.getState());
    FlightStatsRecorder::getInstance().update(ada.getState());

#ifdef HILSimulation
    // useful only for hil testing
    updateData(ada.getState());
#endif  // HILSimulation
}

void ADAController::calibrate()
{
    Stats pressure;

    for (int i = 0; i < CALIBRATION_SAMPLES_COUNT; i++)
    {
        auto data = Sensors::getInstance().getMS5803LastSample();
        pressure.add(data.pressure);

        miosix::Thread::sleep(CALIBRATION_SLEEP_TIME);
    }

    // Set the pressure and temperature reference
    ReferenceValues reference = ada.getReferenceValues();
    reference.refPressure     = pressure.getStats().mean;

    // Update the algorithm reference values
    {
        miosix::PauseKernelLock l;
        ada.setReferenceValues(reference);
        ada.setKalmanConfig(getADAKalmanConfig());
    }

    EventBroker::getInstance().post(ADA_READY, TOPIC_ADA);
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
    reference.refAltitude     = altitude;
    ada.setReferenceValues(reference);
}

void ADAController::setReferenceTemperature(float temperature)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock l;

    ReferenceValues reference = ada.getReferenceValues();
    reference.refTemperature  = temperature;
    ada.setReferenceValues(reference);
}

void ADAController::setReferenceValues(const ReferenceValues reference)
{
    ada.setReferenceValues(reference);
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

ReferenceValues ADAController::getReferenceValues()
{
    return ada.getReferenceValues();
}

float ADAController::getDeploymentAltitude() { return deploymentAltitude; }

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

            // Calibrate the ADA
            calibrate();

            return;
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
        case ADA_FORCE_START:
        case FLIGHT_ARMED:
        {
            return transition(&ADAController::state_armed);
        }
        case FLIGHT_MISSION_TIMEOUT:
        {
            return transition(&ADAController::state_landed);
        }
    }
}

void ADAController::state_armed(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(ADAControllerState::ARMED);
        }
        case ADA_FORCE_STOP:
        case FLIGHT_DISARMED:
        {
            return transition(&ADAController::state_ready);
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

#ifdef HILSimulation
void ADAController::setUpdateDataFunction(
    std::function<void(Boardcore::ADAState)> updateData)
{
    this->updateData = updateData;
}
#endif

ADAController::ADAController()
    : FSM(&ADAController::state_idle), ada(getADAKalmanConfig())
{
    EventBroker::getInstance().subscribe(this, TOPIC_ADA);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);

    ada.setReferenceValues(defaultReferenceValues);
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
            ADA::KalmanFilter::CVectorN(ada.getReferenceValues().refPressure, 0,
                                        0)};
}

}  // namespace Main
