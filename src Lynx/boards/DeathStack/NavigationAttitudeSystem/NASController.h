/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

#pragma once

#include <LoggerService/LoggerService.h>
#include <NavigationAttitudeSystem/NAS.h>
#include <NavigationAttitudeSystem/NASCalibrator.h>
#include <NavigationAttitudeSystem/NASData.h>
#include <System/StackLogger.h>
#include <diagnostic/PrintLogger.h>
#include <events/Events.h>
#include <events/FSM.h>
#include <miosix.h>
#include <sensors/Sensor.h>

using miosix::FastMutex;
using miosix::Lock;

namespace DeathStackBoard
{

/**
 * @brief Navigatioin system state machine
 */
template <typename IMU, typename Press, typename GPS>
class NASController : public Boardcore::FSM<NASController<IMU, Press, GPS>>
{
    using NASCtrl = NASController<IMU, Press, GPS>;
    using NASFsm  = Boardcore::FSM<NASCtrl>;

    static_assert(
        Boardcore::checkIfProduces<Boardcore::Sensor<IMU>,
                                   Boardcore::AccelerometerData>::value,
        "Template argument must be a sensor that produces accelerometer data.");
    static_assert(
        Boardcore::checkIfProduces<Boardcore::Sensor<IMU>,
                                   Boardcore::GyroscopeData>::value,
        "Template argument must be a sensor that produces gyroscope data.");
    static_assert(
        Boardcore::checkIfProduces<Boardcore::Sensor<IMU>,
                                   Boardcore::MagnetometerData>::value,
        "Template argument must be a sensor that produces magnetometer data.");
    static_assert(
        Boardcore::checkIfProduces<Boardcore::Sensor<Press>,
                                   Boardcore::PressureData>::value,
        "Template argument must be a sensor that produces pressure data.");
    static_assert(Boardcore::checkIfProduces<Boardcore::Sensor<GPS>,
                                             Boardcore::GPSData>::value,
                  "Template argument must be a sensor that produces GPS data.");

public:
    NASController(Boardcore::Sensor<IMU>& imu, Boardcore::Sensor<Press>& baro,
                  Boardcore::Sensor<GPS>& gps);
    ~NASController();

    void state_idle(const Boardcore::Event& ev);
    void state_calibrating(const Boardcore::Event& ev);
    void state_ready(const Boardcore::Event& ev);
    void state_active(const Boardcore::Event& ev);
    void state_end(const Boardcore::Event& ev);

    void setReferenceTemperature(float t);
    void setInitialOrientation(float roll, float pitch, float yaw);
    void setInitialCoordinates(float latitude, float longitude);
    void setReferenceAltitude(float altitude);

    void update();

    Boardcore::Sensor<NASData>& getNAS() { return nas; }

private:
    void finalizeCalibration();

    /**
     * @brief Update and log NAS FSM state
     */
    void logStatus(NASState state);
    void logData();

    NASStatus status;

    FastMutex mutex;
    NASCalibrator calibrator;

    Boardcore::Sensor<IMU>& imu;
    Boardcore::Sensor<Press>& barometer;
    Boardcore::Sensor<GPS>& gps;

    NAS<IMU, Press, GPS> nas;

    LoggerService& logger;
    Boardcore::PrintLogger log =
        Boardcore::Logging::getLogger("deathstack.fsm.nas");

    uint64_t last_gps_timestamp   = 0;
    uint64_t last_accel_timestamp = 0;
    uint64_t last_mag_timestamp   = 0;
    uint64_t last_press_timestamp = 0;
};

template <typename IMU, typename Press, typename GPS>
NASController<IMU, Press, GPS>::NASController(Boardcore::Sensor<IMU>& imu,
                                              Boardcore::Sensor<Press>& baro,
                                              Boardcore::Sensor<GPS>& gps)
    : NASFsm(&NASCtrl::state_idle),
      calibrator(NASConfigs::CALIBRATION_N_SAMPLES), imu(imu), barometer(baro),
      gps(gps), nas(imu, baro, gps), logger(LoggerService::getInstance())
{
    memset(&status, 0, sizeof(NASStatus));
    sEventBroker.subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker.subscribe(this, TOPIC_NAS);

    status.state = NASState::IDLE;
}

template <typename IMU, typename Press, typename GPS>
NASController<IMU, Press, GPS>::~NASController()
{
    sEventBroker.unsubscribe(this);
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

                if (imu_data.accelerationTimestamp != last_accel_timestamp)
                {
                    last_accel_timestamp = imu_data.accelerationTimestamp;
                    calibrator.addAccelSample(imu_data.accelerationX,
                                              imu_data.accelerationY,
                                              imu_data.accelerationZ);
                }

                if (imu_data.magneticFieldTimestamp != last_mag_timestamp)
                {
                    last_mag_timestamp = imu_data.magneticFieldTimestamp;
                    calibrator.addMagSample(imu_data.magneticFieldX,
                                            imu_data.magneticFieldY,
                                            imu_data.magneticFieldZ);
                }

                // Add samples to the calibration
                if (press_data.pressureTimestamp != last_press_timestamp)
                {
                    last_press_timestamp = press_data.pressureTimestamp;
                    calibrator.addBaroSample(press_data.pressure);
                }

                if (gps_data.fix == true &&
                    gps_data.gpsTimestamp != last_gps_timestamp)
                {
                    last_gps_timestamp = gps_data.gpsTimestamp;
                    calibrator.addGPSSample(gps_data.latitude,
                                            gps_data.longitude);
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

        LOG_INFO(log, "Finalized calibration and TRIAD");

        sEventBroker.post({EV_NAS_READY}, TOPIC_NAS);
    }
}

template <typename IMU, typename Press, typename GPS>
void NASController<IMU, Press, GPS>::state_idle(const Boardcore::Event& ev)
{
    switch (ev.code)
    {
        case Boardcore::EV_ENTRY:
        {
            LOG_DEBUG(log, "Entering state idle");
            logStatus(NASState::IDLE);
            break;
        }
        case Boardcore::EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state idle");
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
void NASController<IMU, Press, GPS>::state_calibrating(
    const Boardcore::Event& ev)
{
    switch (ev.code)
    {
        case Boardcore::EV_ENTRY:
        {
            LOG_DEBUG(log, "Entering state calibrating");
            logStatus(NASState::CALIBRATING);

            {
                Lock<FastMutex> l(mutex);
                calibrator.reset();
                nas.resetReferenceValues();
            }

            break;
        }
        case Boardcore::EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state calibrating");
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
void NASController<IMU, Press, GPS>::state_ready(const Boardcore::Event& ev)
{
    switch (ev.code)
    {
        case Boardcore::EV_ENTRY:
        {
            LOG_DEBUG(log, "Entering state ready");
            logStatus(NASState::READY);
            break;
        }
        case Boardcore::EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state ready");
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
void NASController<IMU, Press, GPS>::state_active(const Boardcore::Event& ev)
{
    switch (ev.code)
    {
        case Boardcore::EV_ENTRY:
        {
            LOG_DEBUG(log, "Entering state active");
            logStatus(NASState::ACTIVE);
            break;
        }
        case Boardcore::EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state active");
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
void NASController<IMU, Press, GPS>::state_end(const Boardcore::Event& ev)
{
    switch (ev.code)
    {
        case Boardcore::EV_ENTRY:
        {
            LOG_DEBUG(log, "Entering state end");
            logStatus(NASState::END);
            break;
        }
        case Boardcore::EV_EXIT:
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

template <typename IMU, typename Press, typename GPS>
void NASController<IMU, Press, GPS>::setReferenceTemperature(float t)
{
    if (status.state == NASState::CALIBRATING ||
        status.state == NASState::READY)
    {
        Lock<FastMutex> l(mutex);
        calibrator.setReferenceTemperature(t);
        logger.log(calibrator.getReferenceValues());
    }

    finalizeCalibration();
}

template <typename IMU, typename Press, typename GPS>
void NASController<IMU, Press, GPS>::setInitialOrientation(float roll,
                                                           float pitch,
                                                           float yaw)
{
    if (status.state == NASState::READY)
    {
        nas.setInitialOrientation(roll, pitch, yaw);
        logData();
    }

    finalizeCalibration();
}

template <typename IMU, typename Press, typename GPS>
void NASController<IMU, Press, GPS>::setInitialCoordinates(float latitude,
                                                           float longitude)
{
    if (status.state == NASState::CALIBRATING ||
        status.state == NASState::READY)
    {
        Lock<FastMutex> l(mutex);
        calibrator.setReferenceCoordinates(latitude, longitude);
        logData();
    }

    finalizeCalibration();
}

template <typename IMU, typename Press, typename GPS>
void NASController<IMU, Press, GPS>::setReferenceAltitude(float altitude)
{
    if (status.state == NASState::CALIBRATING ||
        status.state == NASState::READY)
    {
        Lock<FastMutex> l(mutex);
        calibrator.setReferenceAltitude(altitude);
        logData();
    }

    finalizeCalibration();
}

template <typename IMU, typename Press, typename GPS>
void NASController<IMU, Press, GPS>::logStatus(NASState state)
{
    status.timestamp = Boardcore::TimestampTimer::getTimestamp();
    status.state     = state;
    logger.log(status);

    Boardcore::StackLogger::getInstance().updateStack(THID_NAS_FSM);
}

template <typename IMU, typename Press, typename GPS>
void NASController<IMU, Press, GPS>::logData()
{
    NASKalmanState kalman_state = nas.getKalmanState();
    kalman_state.timestamp      = Boardcore::TimestampTimer::getTimestamp();
    logger.log(kalman_state);
    logger.log(nas.getLastSample());
}

}  // namespace DeathStackBoard
