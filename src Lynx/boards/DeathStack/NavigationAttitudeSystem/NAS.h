/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Alessandro Del Duca, Luca Conterio, Marco Cella
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

/* STATE VECTOR:     px
 *                   py
 *                   pz
 *                   vx
 *                   vy
 *                   vz
 *            x  =   qx
 *                   qy
 *                   qz
 *                   qw
 *                   bx
 *                   by
 *                   bz
 */

#pragma once

#include <NavigationAttitudeSystem/ExtendedKalmanEigen.h>
#include <NavigationAttitudeSystem/InitStates.h>
#include <NavigationAttitudeSystem/NASData.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/timer/TimestampTimer.h>
#include <math/SkyQuaternion.h>
#include <sensors/Sensor.h>

namespace DeathStackBoard
{

template <typename IMU, typename Press, typename GPS>
class NAS : public Boardcore::Sensor<NASData>
{

public:
    NAS(Boardcore::Sensor<IMU>& imu, Boardcore::Sensor<Press>& baro,
        Boardcore::Sensor<GPS>& gps);

    /**
     * @brief Initialization of the state vector before the liftoff
     *
     * @return boolean indicating if the operation succeded or not
     */
    bool init();

    /**
     * @brief Self-test here does nothing.
     */
    bool selfTest();

    /**
     * @brief Samples the sensors' readings, filters them through the EKF
     *        and computes the state vector
     *
     * @return Struct with the states used by the airbrakes subsystem
     */
    NASData sampleImpl();

    /**
     * @return Struct containing initial positions and orientation estimated
     *         using the triad algorithm.
     */
    NASTriadResult getTriadResult();

    /**
     * @return Struct containing the kalman state.
     */
    NASKalmanState getKalmanState();

    /**
     * @param ref_v Struct of NASReferenceValues to be set
     */
    void setReferenceValues(const NASReferenceValues& ref_v);

    const NASReferenceValues& getReferenceValues();

    void resetReferenceValues();

    void setInitialOrientation(float roll, float pitch, float yaw);

private:
    /**
     * @brief Copy the kalman state information in the NASData output struct.
     */
    void updateNASData();

    /**
     * @brief Convert GPS coordinates to NED frame.
     */
    Eigen::Vector3f geodetic2NED(const Eigen::Vector3f& gps_data);

    Boardcore::SkyQuaternion quat; /**< Auxiliary functions for quaternions */

    Eigen::Matrix<float, NASConfigs::N, 1> x; /**< Kalman state vector */

    NASData nas_data;

    Sensor<IMU>& imu;
    Sensor<Press>& barometer;
    Sensor<GPS>& gps;

    ExtendedKalmanEigen filter;
    InitStates states_init;
    NASReferenceValues ref_values;
    Eigen::Vector3f triad_result_eul;

    uint64_t last_gps_timestamp   = 0;
    uint64_t last_accel_timestamp = 0;
    uint64_t last_gyro_timestamp  = 0;
    uint64_t last_mag_timestamp   = 0;
    uint64_t last_press_timestamp = 0;

    bool initialized = false;

    Boardcore::PrintLogger log =
        Boardcore::Logging::getLogger("deathstack.fsm.nas");

#ifdef DEBUG
    unsigned int counter = 0;
#endif
};

template <typename IMU, typename Press, typename GPS>
NAS<IMU, Press, GPS>::NAS(Sensor<IMU>& imu, Sensor<Press>& baro,
                          Sensor<GPS>& gps)
    : imu(imu), barometer(baro), gps(gps)
{
    x = Eigen::Matrix<float, NASConfigs::N, 1>::Zero();
}

template <typename IMU, typename Press, typename GPS>
bool NAS<IMU, Press, GPS>::init()
{
    states_init.positionInit(ref_values.ref_pressure, ref_values.msl_pressure,
                             ref_values.msl_temperature);

    states_init.velocityInit();

    Eigen::Vector3f acc_init(ref_values.ref_accel_x, ref_values.ref_accel_y,
                             ref_values.ref_accel_z);
    Eigen::Vector3f mag_init(ref_values.ref_mag_x, ref_values.ref_mag_y,
                             ref_values.ref_mag_z);

    triad_result_eul = states_init.triad(acc_init, mag_init);

    states_init.biasInit();

    x = states_init.getInitX();

    filter.setX(x);

    updateNASData();

#ifdef DEBUG
    Eigen::Vector4f qua(x(6), x(7), x(8), x(9));
    Eigen::Vector3f e = quat.quat2eul(qua);

    LOG_DEBUG(
        log,
        "Init state vector: \n px: {:.2f} \n py: {:.2f} \n pz: {:.2f} \n vx: "
        "{:.2f} \n "
        "vy: {:.2f} \n vz: {:.2f} \n roll: {:.2f} "
        "\n pitch: {:.2f} \n yaw: {:.2f}",
        x(0), x(1), x(2), x(3), x(4), x(5), e(0), e(1), e(2));

#endif

    initialized = true;

    LoggerService::getInstance().log(getTriadResult());

    return initialized;
}

template <typename IMU, typename Press, typename GPS>
bool NAS<IMU, Press, GPS>::selfTest()
{
    return true;
}

template <typename IMU, typename Press, typename GPS>
NASData NAS<IMU, Press, GPS>::sampleImpl()
{
    if (!initialized)
    {
        return NASData{};
    }

    IMU imu_data        = imu.getLastSample();
    GPS gps_data        = gps.getLastSample();
    Press pressure_data = barometer.getLastSample();

    // update ekf with new accel and gyro measures
    if (imu_data.accelerationTimestamp != last_accel_timestamp &&
        imu_data.angularSpeedTimestamp != last_gyro_timestamp)
    {
        last_accel_timestamp = imu_data.accelerationTimestamp;
        last_gyro_timestamp  = imu_data.angularSpeedTimestamp;

        Eigen::Vector3f accel_readings(imu_data.accelerationX,
                                       imu_data.accelerationY,
                                       imu_data.accelerationZ);
        filter.predict(accel_readings);

        Eigen::Vector3f gyro_readings(imu_data.angularSpeedX,
                                      imu_data.angularSpeedY,
                                      imu_data.angularSpeedZ);
        filter.predictMEKF(gyro_readings);
    }

    // check if new pressure data is available
    if (pressure_data.pressureTimestamp != last_press_timestamp)
    {
        last_press_timestamp = pressure_data.pressureTimestamp;

        filter.correctBaro(pressure_data.pressure, ref_values.msl_pressure,
                           ref_values.msl_temperature);
    }

    // check if new gps data is available and the gps has fix
    if (gps_data.gpsTimestamp != last_gps_timestamp && gps_data.fix == true)
    {
        last_gps_timestamp = gps_data.gpsTimestamp;

        Eigen::Vector3f gps_readings(gps_data.latitude, gps_data.longitude,
                                     gps_data.height);
        Eigen::Vector3f gps_ned = geodetic2NED(gps_readings);

        Eigen::Vector4f pos_vel(gps_ned(0), gps_ned(1), gps_data.velocityNorth,
                                gps_data.velocityEast);
        filter.correctGPS(pos_vel, gps_data.satellites);
    }

    // check if new magnetometer data is available
    if (imu_data.magneticFieldTimestamp != last_mag_timestamp)
    {
        Eigen::Vector3f mag_readings(imu_data.magneticFieldX,
                                     imu_data.magneticFieldY,
                                     imu_data.magneticFieldZ);

        if (mag_readings.norm() < NASConfigs::EMF * NASConfigs::JAMMING_FACTOR)
        {
            last_mag_timestamp = imu_data.magneticFieldTimestamp;

            mag_readings.normalize();
            filter.correctMEKF(mag_readings);
        }
    }

    // update states
    x = filter.getState();

    updateNASData();

#ifdef DEBUG
    if (counter == 50)
    {
        // TRACE("[NAS] x(2) : %.2f - z_init : %.2f \n", x(2), z_init);
        LOG_DEBUG(log, "z : {:.2f} - vz : {:.2f} - vMod : {:.2f}", nas_data.z,
                  nas_data.vz, nas_data.vMod);

        counter = 0;
    }
    else
    {
        counter++;
    }
#endif

    return nas_data;
}

template <typename IMU, typename Press, typename GPS>
NASTriadResult NAS<IMU, Press, GPS>::getTriadResult()
{
    Eigen::Matrix<float, NASConfigs::N, 1> state = states_init.getInitX();
    // Eigen::Vector3f e = quat.quat2eul({state(6), state(7), state(8),
    // state(9)});

    NASTriadResult result;
    result.x     = state(0);
    result.y     = state(1);
    result.z     = -state(2);
    result.roll  = triad_result_eul(0);  // e(0);
    result.pitch = triad_result_eul(1);  // e(1);
    result.yaw   = triad_result_eul(2);  // e(2);

    return result;
}

template <typename IMU, typename Press, typename GPS>
NASKalmanState NAS<IMU, Press, GPS>::getKalmanState()
{
    return NASKalmanState{nas_data.timestamp, x};
}

template <typename IMU, typename Press, typename GPS>
void NAS<IMU, Press, GPS>::setReferenceValues(const NASReferenceValues& ref_v)
{
    this->ref_values = ref_v;
}

template <typename IMU, typename Press, typename GPS>
const NASReferenceValues& NAS<IMU, Press, GPS>::getReferenceValues()
{
    return ref_values;
}

template <typename IMU, typename Press, typename GPS>
void NAS<IMU, Press, GPS>::resetReferenceValues()
{
    ref_values = NASReferenceValues{};
}

template <typename IMU, typename Press, typename GPS>
void NAS<IMU, Press, GPS>::setInitialOrientation(float roll, float pitch,
                                                 float yaw)
{
    Eigen::Vector4f q     = quat.eul2quat({yaw, pitch, roll});
    x(NASConfigs::NL)     = q(0);
    x(NASConfigs::NL + 1) = q(1);
    x(NASConfigs::NL + 2) = q(2);
    x(NASConfigs::NL + 3) = q(3);
    LOG_INFO(log, "Initial orientation set to : ({:.2f}, {:.2f}, {:.2f})", roll,
             pitch, yaw);
    LOG_DEBUG(log,
              "State vector: \n px: {:.2f} \n py: {:.2f} \n pz: {:.2f} \n vx: "
              "{:.2f} \n "
              "vy: {:.2f} \n vz: {:.2f} \n roll: {:.2f} \n pitch: {:.2f} \n "
              "yaw: {:.2f} \n "
              "q1: {:.2f} \n q2: {:.2f} \n q3: {:.2f} \n q4 : {:.2f}",
              x(0), x(1), x(2), x(3), x(4), x(5), roll, pitch, yaw, x(6), x(7),
              x(8), x(9));
}

template <typename IMU, typename Press, typename GPS>
void NAS<IMU, Press, GPS>::updateNASData()
{
    nas_data.timestamp = Boardcore::TimestampTimer::getTimestamp();

    nas_data.x = x(0);
    nas_data.y = x(1);
    nas_data.z =
        -x(2) -
        ref_values
            .ref_altitude;  // Negative sign because we're working in the NED
                            // frame but we want a positive altitude as output.
    nas_data.vx = x(3);
    nas_data.vy = x(4);
    nas_data.vz = -x(5);
    nas_data.vMod =
        sqrtf(nas_data.vx * nas_data.vx + nas_data.vy * nas_data.vy +
              nas_data.vz * nas_data.vz);
}

template <typename IMU, typename Press, typename GPS>
Eigen::Vector3f NAS<IMU, Press, GPS>::geodetic2NED(
    const Eigen::Vector3f& gps_data)
{
    float lat0 = ref_values.ref_latitude * Boardcore::DEGREES_TO_RADIANS;
    float lon0 = ref_values.ref_longitude * Boardcore::DEGREES_TO_RADIANS;
    float lat  = gps_data(0) * Boardcore::DEGREES_TO_RADIANS;
    float lon  = gps_data(1) * Boardcore::DEGREES_TO_RADIANS;
    float h    = gps_data(2);

    float s1      = sin(lat0);
    float c1      = cos(lat0);
    float s2      = sin(lat);
    float c2      = cos(lat);
    float p1      = c1 * cos(lon0);
    float p2      = c2 * cos(lon);
    float q1      = c1 * sin(lon0);
    float q2      = c2 * sin(lon);
    float w1      = 1 / sqrt(1 - NASConfigs::e2 * pow(s1, 2));
    float w2      = 1 / sqrt(1 - NASConfigs::e2 * pow(s2, 2));
    float delta_x = NASConfigs::a * (p2 * w2 - p1 * w1) +
                    (h * p2 - ref_values.ref_altitude * p1);
    float delta_y = NASConfigs::a * (q2 * w2 - q1 * w1) +
                    (h * q2 - ref_values.ref_altitude * q1);
    float delta_z = (1 - NASConfigs::e2) * NASConfigs::a * (s2 * w2 - s1 * w1) +
                    (h * s2 - ref_values.ref_altitude * s1);

    // positions in ENU (east, north, up) frame
    float p_east  = -sin(lon0) * delta_x + cos(lon0) * delta_y;
    float p_north = -sin(lat0) * cos(lon0) * delta_x -
                    sin(lat0) * sin(lon0) * delta_y + cos(lat0) * delta_z;
    float p_up = cos(lat0) * cos(lon0) * delta_x +
                 cos(lat0) * sin(lon0) * delta_y + sin(lat0) * delta_z;

    // positions in NED frame
    Eigen::Vector3f p_ned(p_north, p_east, -p_up);

    return p_ned;
}

}  // namespace DeathStackBoard
