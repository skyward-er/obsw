/*
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Conterio
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

#include <miosix.h>

#include "NAS.h"
#include "NASCalibrator.h"
#include "NASData.h"
#include "events/Events.h"
#include "events/FSM.h"
#include "sensors/Sensor.h"

#include "LoggerService/LoggerService.h"
#include "diagnostic/PrintLogger.h"

using miosix::FastMutex;
using miosix::Lock;

namespace DeathStackBoard
{

using namespace NASConfigs;

/**
 * @brief Navigatioin system state machine
 */
template <typename IMU, typename Press, typename GPS>
class NASController : public FSM<NASController<IMU, Press, GPS>>
{
    using NASCtrl = NASController<IMU, Press, GPS>;
    using NASFsm  = FSM<NASCtrl>;

    static_assert(
        checkIfProduces<Sensor<IMU>, AccelerometerData>::value,
        "Template argument must be a sensor that produces accelerometer data.");
    static_assert(
        checkIfProduces<Sensor<IMU>, GyroscopeData>::value,
        "Template argument must be a sensor that produces gyroscope data.");
    static_assert(
        checkIfProduces<Sensor<IMU>, MagnetometerData>::value,
        "Template argument must be a sensor that produces magnetometer data.");
    static_assert(
        checkIfProduces<Sensor<Press>, PressureData>::value,
        "Template argument must be a sensor that produces pressure data.");
    static_assert(checkIfProduces<Sensor<GPS>, GPSData>::value,
                  "Template argument must be a sensor that produces GPS data.");

public:
    NASController(Sensor<IMU>& imu, Sensor<Press>& baro, Sensor<GPS>& gps);
    ~NASController();

    void state_idle(const Event& ev);
    void state_calibrating(const Event& ev);
    void state_ready(const Event& ev);
    void state_active(const Event& ev);
    void state_end(const Event& ev);

    void update();

    void setInitialOrientation(float roll, float pitch, float yaw);

    Sensor<NASData>& getNAS() { return nas; }

private:
    void finalizeCalibration();

    void logStatus(NASState state);  // Update and log NAS FSM state
    void logData();

    NASStatus status;

    /* --- CALIBRATION --- */
    FastMutex mutex;
    NASCalibrator calibrator;

    Sensor<IMU>& imu;
    Sensor<Press>& barometer;
    Sensor<GPS>& gps;

    NAS<IMU, Press, GPS> nas;

    LoggerService& logger;
    PrintLogger log = Logging::getLogger("deathstack.fsm.nas");

    uint64_t last_gps_timestamp   = 0;
    uint64_t last_accel_timestamp = 0;
    uint64_t last_mag_timestamp   = 0;
    uint64_t last_press_timestamp = 0;
};

template <typename IMU, typename Press, typename GPS>
NASController<IMU, Press, GPS>::NASController(Sensor<IMU>& imu,
                                              Sensor<Press>& baro,
                                              Sensor<GPS>& gps)
    : NASFsm(&NASCtrl::state_idle), calibrator(CALIBRATION_N_SAMPLES), imu(imu),
      barometer(baro), gps(gps), nas(imu, baro, gps),
      logger(*(LoggerService::getInstance()))
{
    memset(&status, 0, sizeof(NASStatus));
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_NAS);

    status.state = NASState::IDLE;
}

template <typename IMU, typename Press, typename GPS>
NASController<IMU, Press, GPS>::~NASController()
{
    sEventBroker->unsubscribe(this);
}

template <typename IMU, typename Press, typename GPS>
void NASController<IMU, Press, GPS>::update()
{
    IMU imu_data     = imu.getLastSample();
    Press press_data = barometer.getLastSample();
    GPS gps_data     = gps.getLastSample();

    NASState state = status.state;

    switch (state)
    {
        case NASState::IDLE:
        {
            break;
        }
        case NASState::CALIBRATING:
        {
            bool end_calib = false;
            {
                Lock<FastMutex> l(mutex);

                // Add samples to the calibration
                // if (press_data.press_timestamp > last_press_timestamp)
                {
                    last_press_timestamp = press_data.press_timestamp;
                    calibrator.addBaroSample(press_data.press);
                }

                if (/*gps_data.gps_timestamp > last_gps_timestamp &&*/
                    gps_data.fix == true)
                {
                    last_gps_timestamp = gps_data.gps_timestamp;
                    calibrator.addGPSSample(gps_data.latitude,
                                            gps_data.longitude);
                }

                // if (imu_data.accel_timestamp > last_accel_timestamp)
                {
                    last_accel_timestamp = imu_data.accel_timestamp;
                    calibrator.addAccelSample(
                        imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
                }

                // if (imu_data.mag_timestamp > last_mag_timestamp)
                {
                    last_mag_timestamp = imu_data.mag_timestamp;
                    calibrator.addMagSample(imu_data.mag_x, imu_data.mag_y,
                                            imu_data.mag_z);
                }

                // Save the state of calibration to release mutex
                end_calib = calibrator.calibIsComplete();
            }

            if (end_calib)
            {
                // If samples are enough
                finalizeCalibration();
            }

            break;
        }
        case NASState::READY:
        {
            // simply log data
            break;
        }
        case NASState::ACTIVE:
        {
            nas.sample();
            logData();
            break;
        }
        case NASState::END:
        {
            break;
        }
        default:
        {
            break;
        }
    }
}

template <typename IMU, typename Press, typename GPS>
void NASController<IMU, Press, GPS>::finalizeCalibration()
{
    Lock<FastMutex> l(mutex);

    if (calibrator.calibIsComplete() &&
        nas.getReferenceValues() != calibrator.getReferenceValues())
    {
        nas.setReferenceValues(calibrator.getReferenceValues());

        // initialize NAS and execute TRIAD
        nas.init();

        // Log all the data after init, reference values, kalman state and also
        // the triad result (with euler angles)
        logger.log(calibrator.getReferenceValues());
        logger.log(nas.getTriadResult());
        logData();

        LOG_INFO(log, "Finalized calibration and TRIAD \n");

        sEventBroker->post({EV_NAS_READY}, TOPIC_NAS);
    }
}

template <typename IMU, typename Press, typename GPS>
void NASController<IMU, Press, GPS>::state_idle(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            LOG_DEBUG(log, "Entering state idle\n");
            logStatus(NASState::IDLE);
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state idle\n");
            break;
        }
        case EV_CALIBRATE_NAS:
        {
            this->transition(&NASCtrl::state_calibrating);
            break;
        }
        default:
        {
            break;
        }
    }
}

template <typename IMU, typename Press, typename GPS>
void NASController<IMU, Press, GPS>::state_calibrating(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            LOG_DEBUG(log, "Entering state calibrating\n");
            logStatus(NASState::CALIBRATING);

            {
                Lock<FastMutex> l(mutex);
                calibrator.reset();
                nas.resetReferenceValues();
            }

            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state calibrating\n");
            break;
        }
        case EV_CALIBRATE_NAS:
        {
            this->transition(&NASCtrl::state_calibrating);
            break;
        }
        case EV_NAS_READY:
        {
            this->transition(&NASCtrl::state_ready);
            break;
        }
        default:
        {
            break;
        }
    }
}

template <typename IMU, typename Press, typename GPS>
void NASController<IMU, Press, GPS>::state_ready(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            LOG_DEBUG(log, "Entering state ready\n");
            logStatus(NASState::READY);
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state ready\n");
            break;
        }
        case EV_LIFTOFF:
        {
            this->transition(&NASCtrl::state_active);
            break;
        }
        case EV_CALIBRATE_NAS:
        {
            this->transition(&NASCtrl::state_calibrating);
            break;
        }
        default:
        {
            break;
        }
    }
}

template <typename IMU, typename Press, typename GPS>
void NASController<IMU, Press, GPS>::state_active(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            LOG_DEBUG(log, "Entering state active\n");
            logStatus(NASState::ACTIVE);
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state active\n");
            break;
        }
        case EV_LANDED:
        {
            this->transition(&NASCtrl::state_end);
            break;
        }
        default:
        {
            break;
        }
    }
}

template <typename IMU, typename Press, typename GPS>
void NASController<IMU, Press, GPS>::state_end(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            LOG_DEBUG(log, "Entering state end\n");
            logStatus(NASState::END);
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state end\n");
            break;
        }

        default:
        {
            break;
        }
    }
}

template <typename IMU, typename Press, typename GPS>
void NASController<IMU, Press, GPS>::setInitialOrientation(float roll,
                                                           float pitch,
                                                           float yaw)
{
    if (status.state == NASState::READY)
    {
        nas.setInitialOrientation(roll, pitch, yaw);
    }
}

/* --- LOGGER --- */

template <typename IMU, typename Press, typename GPS>
void NASController<IMU, Press, GPS>::logStatus(NASState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;
    logger.log(status);

    StackLogger::getInstance()->updateStack(THID_NAS_FSM);
}

template <typename IMU, typename Press, typename GPS>
void NASController<IMU, Press, GPS>::logData()
{
    logger.log(nas.getKalmanState());
    logger.log(nas.getLastSample());
}
}  // namespace DeathStackBoard