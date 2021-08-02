/* Copyright (c) 2018-2021 Skyward Experimental Rocketry
 * Authors: Luca Mozzarelli, Luca Conterio
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

#include <ADA/ADAData.h>
#include <configs/ADAConfig.h>
#include <events/EventBroker.h>
#include <events/Events.h>
#include <events/FSM.h>
#include <miosix.h>
#include <utils/aero/AeroUtils.h>

#include <diagnostic/PrintLogger.h>

#include "ADA/ADA.h"
#include "ADA/ADACalibrator.h"
#include "Debug.h"
#include "LoggerService/LoggerService.h"
#include "System/StackLogger.h"
#include "TimestampTimer.h"
#include "sensors/SensorData.h"

using miosix::FastMutex;
using miosix::Lock;

namespace DeathStackBoard
{

using namespace ADAConfigs;

template <typename Press, typename GPS>
class ADAController : public FSM<ADAController<Press, GPS>>
{
    using ADACtrl = ADAController<Press, GPS>;
    using ADAFsm  = FSM<ADAController<Press, GPS>>;

    static_assert(
        checkIfProduces<Sensor<Press>, PressureData>::value,
        "Template argument must be a sensor that produces pressure data.");
    static_assert(checkIfProduces<Sensor<GPS>, GPSData>::value,
                  "Template argument must be a sensor that produces GPS data.");

public:
    ADAController(Sensor<Press>& barometer, Sensor<GPS>& gps);

    ~ADAController();

    /* --- SENSOR UPDATE METHODS --- */
    /*
     * It's critical that this methods are called at regualar intervals during
     * the flight. Call frequency is defined in configs/ADAConfig.h The behavior
     * of this functions changes depending on the ADA state
     */
    void update();

    /* --- TC --- */
    /**
     * Sets the reference temperature to be used to calibrate the altimeter
     * @param ref_temp Reference temperature in degrees Celsisus
     */
    void setReferenceTemperature(float ref_temp);

    /**
     * Sets the reference altitude to be used to calibrate the altimeter
     * @param ref_alt Reference altitude in meters above mean sea level
     */
    void setReferenceAltitude(float ref_alt);

    /**
     * Sets the deployment altitude
     * @param dpl_alt Deployment altitude in meters above GROUND level
     */
    void setDeploymentAltitude(float dpl_alt);

    /**
     * ADAController status
     * @returns A struct containing the timestamp, the ADA FSM state and
     * several flags
     */
    ADAControllerStatus getStatus() { return status; }

    /**
     * @returns The current ADAData structure
     */
    ADAData getADAData() { return ada.getADAData(); }

    /**
     * @returns The current ADAReferenceValues structure
     */
    ADAReferenceValues getReferenceValues() { return ada.getReferenceValues(); }

private:
    /* --- FSM STATES --- */
    void state_init(const Event& ev);
    void state_idle(const Event& ev);
    void state_calibrating(const Event& ev);
    void state_ready(const Event& ev);
    void state_shadowMode(const Event& ev);
    void state_active(const Event& ev);
    void state_pressureStabilization(const Event& ev);
    void state_drogueDescent(const Event& ev);
    void state_end(const Event& ev);

    void updateBaroAccordingToState(float pressure);

    void finalizeCalibration();

    void logStatus(ADAState state);  // Update and log ADA FSM state
    void logStatus();  // Log the ADA FSM state without updating it

    void logData(const ADAKalmanState& s, const ADAData& d);

    uint16_t shadow_delayed_event_id =
        0;  // Event id to store shadow mode timeout

    uint16_t pressure_delayed_event_id =
        0;  // Event id to store pressure stabilization timeout

    ADAControllerStatus status;  // ADA status: timestamp + state

    /* --- CALIBRATION --- */
    FastMutex calibrator_mutex;
    ADACalibrator calibrator;

    /* --- ALGORITHM --- */
    ADA ada;

    Sensor<Press>& barometer;
    Sensor<GPS>& gps;

    uint64_t last_press_timestamp = 0;
    uint64_t last_gps_timestamp   = 0;

    float deployment_altitude    = DEFAULT_DEPLOYMENT_ALTITUDE;
    bool deployment_altitude_set = false;

    unsigned int n_samples_apogee_detected =
        0; /**< Number of consecutive samples in which apogee is detected */
    unsigned int n_samples_deployment_detected =
        0; /**<  Number of consecutive samples in which dpl altitude is detected
            */
    unsigned int n_samples_abk_disable_detected =
        0; /**<  Number of consecutive samples for abk disable */

    LoggerService& logger = *(LoggerService::getInstance());  // Logger
    PrintLogger log = Logging::getLogger("deathstack.fms.ada");
};

/* --- LIFE CYCLE --- */
template <typename Press, typename GPS>
ADAController<Press, GPS>::ADAController(Sensor<Press>& barometer,
                                         Sensor<GPS>& gps)
    : ADAFsm(&ADACtrl::state_idle, STACK_MIN_FOR_SKYWARD, ADA_PRIORITY),
      ada(ADAReferenceValues{}), barometer(barometer), gps(gps)
{
    // Subscribe to topics
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_ADA);

    status.state = ADAState::IDLE;
}

template <typename Press, typename GPS>
ADAController<Press, GPS>::~ADAController()
{
}

/* --- SENSOR UPDATE METHODS --- */
template <typename Press, typename GPS>
void ADAController<Press, GPS>::update()
{
    // if new gps data available, update GPS, regardless of the current state
    GPS gps_data = gps.getLastSample();
    if (gps_data.gps_timestamp > last_gps_timestamp)
    {
        last_gps_timestamp = gps_data.gps_timestamp;

        ada.updateGPS(gps_data.latitude, gps_data.longitude, gps_data.fix);
    }

    // if new pressure data available, update baro, according to current state
    Press press_data = barometer.getLastSample();

    if (press_data.press_timestamp > last_press_timestamp)
    {
        last_press_timestamp = press_data.press_timestamp;

        updateBaroAccordingToState(press_data.press);
    }
}

template <typename Press, typename GPS>
void ADAController<Press, GPS>::updateBaroAccordingToState(float pressure)
{
    ADAState state = status.state;

    switch (state)
    {
        case ADAState::IDLE:
        {
            break;
        }
        case ADAState::CALIBRATING:
        {
            bool end_calib = false;
            {
                Lock<FastMutex> l(calibrator_mutex);

                // Add samples to the calibration
                calibrator.addBaroSample(pressure);

                // Save the state of calibration to release mutex
                end_calib = calibrator.calibIsComplete();
            }

            if (end_calib)
            {
                // If samples are enough and dpl altitude has been set init ada
                finalizeCalibration();
            }
            break;
        }

        case ADAState::READY:
        {
            // Log the altitude & vertical speed but don't use kalman pressure
            // while we are on the ramp
            ADAData d;
            d.timestamp = TimestampTimer::getTimestamp();

            d.msl_altitude = ada.pressureToAltitude(pressure);
            d.vert_speed   = 0;

            ADA::AltitudeDPL ad   = ada.altitudeMSLtoDPL(d.msl_altitude);
            d.dpl_altitude        = ad.altitude;
            d.is_dpl_altitude_agl = ad.is_agl;

            logger.log(d);
            break;
        }

        case ADAState::SHADOW_MODE:
        {
            // Shadow mode state: update kalman, DO NOT send events
            ada.updateBaro(pressure);

            // Check if the vertical speed smaller than the target apogee speed
            if (ada.getVerticalSpeed() < APOGEE_VERTICAL_SPEED_TARGET)
            {
                // Log
                ApogeeDetected apogee{TimestampTimer::getTimestamp(),
                                      status.state};
                logger.log(apogee);
            }

            logData(ada.getKalmanState(), ada.getADAData());

            break;
        }

        case ADAState::ACTIVE:
        {
            ada.updateBaro(pressure);

            // Check if we reached apogee
            if (ada.getVerticalSpeed() < APOGEE_VERTICAL_SPEED_TARGET)
            {
                if (++n_samples_apogee_detected >= APOGEE_N_SAMPLES)
                {
                    // Active state send notifications for apogee
                    sEventBroker->post({EV_ADA_APOGEE_DETECTED}, TOPIC_ADA);
                    status.apogee_reached = true;
                }

                // Log
                ApogeeDetected apogee{TimestampTimer::getTimestamp(),
                                      status.state};
                logger.log(apogee);
            }
            else if (n_samples_apogee_detected != 0)
            {
                n_samples_apogee_detected = 0;
            }

            // Check if we have to disable aerobrakes
            if (ada.getVerticalSpeed() < ABK_DISABLE_VERTICAL_SPEED_TARGET)
            {
                if (++n_samples_abk_disable_detected >= ABK_DISABLE_N_SAMPLES)
                {
                    // Active state send notifications for disabling aerobrakes
                    sEventBroker->post({EV_ADA_DISABLE_ABK}, TOPIC_FLIGHT_EVENTS);
                    status.disable_aerobrakes = true;
                }
            }
            else if (n_samples_abk_disable_detected != 0)
            {
                n_samples_abk_disable_detected = 0;
            }

            logData(ada.getKalmanState(), ada.getADAData());
            break;
        }

        case ADAState::PRESSURE_STABILIZATION:
        {
            // Stabilization state: do not send notifications for target
            // altitude reached, log it
            ada.updateBaro(pressure);

            if (ada.getAltitudeForDeployment().altitude <=
                    deployment_altitude &&
                ada.getAltitudeMsl() <= MAX_DEPLOYMENT_ALTITUDE_MSL)
            {
                if (++n_samples_deployment_detected >= DEPLOYMENT_N_SAMPLES)
                {
                    logger.log(
                        DplAltitudeReached{TimestampTimer::getTimestamp()});
                }
            }
            else if (n_samples_deployment_detected != 0)
            {
                n_samples_deployment_detected = 0;
            }

            logData(ada.getKalmanState(), ada.getADAData());
            break;
        }

        case ADAState::DROGUE_DESCENT:
        {
            // Descent state: send notifications for target altitude reached
            ada.updateBaro(pressure);

            if (ada.getAltitudeForDeployment().altitude <=
                    deployment_altitude &&
                ada.getAltitudeMsl() <= MAX_DEPLOYMENT_ALTITUDE_MSL)
            {
                if (++n_samples_deployment_detected >= DEPLOYMENT_N_SAMPLES)
                {
                    logger.log(
                        DplAltitudeReached{TimestampTimer::getTimestamp()});

                    sEventBroker->post({EV_ADA_DPL_ALT_DETECTED}, TOPIC_ADA);
                }
            }
            else if (n_samples_deployment_detected != 0)
            {
                n_samples_deployment_detected = 0;
            }

            logData(ada.getKalmanState(), ada.getADAData());
            break;
        }
        case ADAState::END:
        {
            // Continue updating the filter for logging & telemetry purposes
            ada.updateBaro(pressure);

            logData(ada.getKalmanState(), ada.getADAData());
            break;
        }
        default:
        {
            LOG_INFO(log, "Update Baro: Unexpected state value ");
            break;
        }
    }
}

/* --- TC --- */
template <typename Press, typename GPS>
void ADAController<Press, GPS>::setReferenceTemperature(float ref_temp)
{
    if (status.state == ADAState::CALIBRATING ||
        status.state == ADAState::READY)
    {
        {
            Lock<FastMutex> l(calibrator_mutex);
            calibrator.setReferenceTemperature(ref_temp);
            logger.log(calibrator.getReferenceValues());
        }

        finalizeCalibration();
    }
}

template <typename Press, typename GPS>
void ADAController<Press, GPS>::setReferenceAltitude(float ref_alt)
{
    if (status.state == ADAState::CALIBRATING ||
        status.state == ADAState::READY)
    {
        {
            Lock<FastMutex> l(calibrator_mutex);
            calibrator.setReferenceAltitude(ref_alt);
            logger.log(calibrator.getReferenceValues());
        }

        finalizeCalibration();
    }
}

template <typename Press, typename GPS>
void ADAController<Press, GPS>::setDeploymentAltitude(float dpl_alt)
{
    if (status.state == ADAState::CALIBRATING ||
        status.state == ADAState::READY)
    {
        {
            Lock<FastMutex> l(calibrator_mutex);

            deployment_altitude     = dpl_alt;
            deployment_altitude_set = true;
        }
        logger.log(TargetDeploymentAltitude{deployment_altitude});

        LOG_INFO(log, "Deployment altitude set to {:.3f} m", dpl_alt);

        finalizeCalibration();
    }
}

/* --- CALIBRATION --- */
template <typename Press, typename GPS>
void ADAController<Press, GPS>::finalizeCalibration()
{
    Lock<FastMutex> l(calibrator_mutex);

    if (calibrator.calibIsComplete() && deployment_altitude_set &&
        ada.getReferenceValues() != calibrator.getReferenceValues())
    {
        // If samples are enough and dpl altitude has been set init ada
        ada = ADA{calibrator.getReferenceValues()};

        LOG_INFO(log, "Finalized calibration\n");

        // ADA READY!
        sEventBroker->post({EV_ADA_READY}, TOPIC_ADA);

        logger.log(calibrator.getReferenceValues());
        logger.log(ada.getKalmanState());
    }
}

/* --- STATES --- */
/**
 * \brief Idle state: the ADA waits for a command to start calibration. This
 * is the initial state.
 */
template <typename Press, typename GPS>
void ADAController<Press, GPS>::state_idle(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            LOG_INFO(log, "Entering state idle");
            logStatus(ADAState::IDLE);
            break;
        }
        case EV_EXIT:
        {
            LOG_INFO(log, "Exiting state idle");
            break;
        }
        case EV_CALIBRATE_ADA:
        {
            this->transition(&ADACtrl::state_calibrating);
            break;
        }
        default:
        {
            break;
        }
    }
}

/**
 * \brief Calibrating state: the ADA calibrates the initial Kalman state.
 *
 * In this state a call to update() will result in a altitude sample being
 * added to the average. The exiting transition to the idle state is
 * triggered at the first sample update after having set the deployment
 * altitude and having reached the minimum number of calibration samples.
 */
template <typename Press, typename GPS>
void ADAController<Press, GPS>::state_calibrating(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            {
                Lock<FastMutex> l(calibrator_mutex);
                calibrator.resetBaro();
            }
            logStatus(ADAState::CALIBRATING);
            LOG_INFO(log, "Entering state calibrating");
            break;
        }
        case EV_EXIT:
        {
            LOG_INFO(log, "Exiting state calibrating");
            break;
        }
        case EV_ADA_READY:
        {
            this->transition(&ADACtrl::state_ready);
            break;
        }
        case EV_CALIBRATE_ADA:
        {
            this->transition(&ADACtrl::state_calibrating);
            break;
        }
        default:
        {
            break;
        }
    }
}

/**
 * \brief Ready state:  ADA is ready and waiting for liftoff
 *
 * In this state a call to update() will have no effect.
 * The exiting transition to the shadow mode state is triggered by the
 * liftoff event.
 */
template <typename Press, typename GPS>
void ADAController<Press, GPS>::state_ready(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            logStatus(ADAState::READY);
            LOG_INFO(log, "Entering state ready");
            break;
        }
        case EV_EXIT:
        {
            LOG_INFO(log, "Exiting state ready");
            break;
        }
        case EV_LIFTOFF:
        {
            this->transition(&ADACtrl::state_shadowMode);
            break;
        }
        case EV_CALIBRATE_ADA:
        {
            this->transition(&ADACtrl::state_calibrating);
            break;
        }
        default:
        {
            break;
        }
    }
}

/**
 * \brief Shadow mode state:  ADA is running and logging apogees detected,
 * but is not generating events
 *
 * In this state a call to update() will trigger a one step update of the
 * kalman filter followed by a check of vertical speed sign. The exiting
 * transition to the active state is triggered by a timeout event.
 */
template <typename Press, typename GPS>
void ADAController<Press, GPS>::state_shadowMode(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            shadow_delayed_event_id =
                sEventBroker->postDelayed<TIMEOUT_ADA_SHADOW_MODE>(
                    {EV_SHADOW_MODE_TIMEOUT}, TOPIC_ADA);
            logStatus(ADAState::SHADOW_MODE);
            LOG_INFO(log, "Entering state shadowMode");
            break;
        }
        case EV_EXIT:
        {
            sEventBroker->removeDelayed(shadow_delayed_event_id);
            LOG_INFO(log, "Exiting state shadowMode");
            break;
        }
        case EV_SHADOW_MODE_TIMEOUT:
        {
            this->transition(&ADACtrl::state_active);
            break;
        }
        default:
        {
            break;
        }
    }
}

/**
 * \brief Active state:  ADA is running and it generates an event whe apogee
 * is detected
 *
 * In this state a call to update() will trigger a one step update of the
 * kalman filter followed by a check of vertical speed sign. The exiting
 * transition to the descent state is triggered by the apogee reached event
 * (NOT self generated!)
 */
template <typename Press, typename GPS>
void ADAController<Press, GPS>::state_active(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            logStatus(ADAState::ACTIVE);
            LOG_INFO(log, "Entering state active");
            break;
        }
        case EV_EXIT:
        {
            LOG_INFO(log, "Exiting state active");
            break;
        }
        case EV_ADA_APOGEE_DETECTED:
        {
            this->transition(&ADACtrl::state_pressureStabilization);
            break;
        }
        default:
        {
            break;
        }
    }
}

template <typename Press, typename GPS>
void ADAController<Press, GPS>::state_pressureStabilization(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            pressure_delayed_event_id =
                sEventBroker->postDelayed<TIMEOUT_ADA_P_STABILIZATION>(
                    {EV_TIMEOUT_PRESS_STABILIZATION}, TOPIC_ADA);
            logStatus(ADAState::PRESSURE_STABILIZATION);
            LOG_INFO(log, "Entering state pressureStabilization");
            break;
        }
        case EV_EXIT:
        {
            sEventBroker->removeDelayed(pressure_delayed_event_id);
            LOG_INFO(log, "Exiting state pressureStabilization");
            break;
        }
        case EV_TIMEOUT_PRESS_STABILIZATION:
        {
            this->transition(&ADACtrl::state_drogueDescent);
            break;
        }
        default:
        {
            break;
        }
    }
}

/**
 * \brief First descent phase state:  ADA is running and it generates an
 * event when a set altitude is reached
 *
 * In this state a call to update() will trigger a one step update of the
 * kalman filter followed by a check of the altitude. The exiting transition
 * to the stop state is triggered by the parachute deployment altitude
 * reached event (NOT self generated!)
 */
template <typename Press, typename GPS>
void ADAController<Press, GPS>::state_drogueDescent(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            logStatus(ADAState::DROGUE_DESCENT);
            LOG_INFO(log, "Entering state drogueDescent");
            n_samples_deployment_detected = 0;
            break;
        }
        case EV_EXIT:
        {
            LOG_INFO(log, "Exiting state drogueDescent");
            break;
        }
        case EV_ADA_DPL_ALT_DETECTED:
        {
            status.dpl_altitude_reached = true;
            logStatus();

            this->transition(&ADACtrl::state_end);
            break;
        }
        default:
        {
            break;
        }
    }
}

/**
 * \brief End state:  ADA is stopped
 *
 * In this state a call to update() will have no effect.
 * This is the final state
 */
template <typename Press, typename GPS>
void ADAController<Press, GPS>::state_end(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            LOG_INFO(log, "Entering state end");
            logStatus(ADAState::END);
            break;
        }
        case EV_EXIT:
        {
            LOG_INFO(log, "Exiting state end");
            break;
        }
        default:
        {
            break;
        }
    }
}

/* --- LOGGER --- */
template <typename Press, typename GPS>
void ADAController<Press, GPS>::logStatus(ADAState state)
{
    status.state = state;
    logStatus();
}

template <typename Press, typename GPS>
void ADAController<Press, GPS>::logStatus()
{
    status.timestamp = TimestampTimer::getTimestamp();
    logger.log(status);

    StackLogger::getInstance()->updateStack(THID_ADA_FSM);
}

template <typename Press, typename GPS>
void ADAController<Press, GPS>::logData(const ADAKalmanState& s,
                                        const ADAData& d)
{
    logger.log(s);
    logger.log(d);
}

}  // namespace DeathStackBoard