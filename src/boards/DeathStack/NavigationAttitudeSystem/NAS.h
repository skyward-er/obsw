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
#include <TimestampTimer.h>
#include <diagnostic/PrintLogger.h>
#include <math/SkyQuaternion.h>
#include <sensors/Sensor.h>

namespace DeathStackBoard
{

using namespace NASConfigs;

template <typename IMU, typename Press, typename GPS>
class NAS : public Sensor<NASData>
{

public:
    NAS(Sensor<IMU>& imu, Sensor<Press>& baro, Sensor<GPS>& gps);

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
    Vector3f geodetic2NED(const Vector3f& gps_data);

    SkyQuaternion quat; /**< Auxiliary functions for quaternions */

    Matrix<float, N, 1> x; /**< Kalman state vector */

    NASData nas_data;

    Sensor<IMU>& imu;
    Sensor<Press>& barometer;
    Sensor<GPS>& gps;

    ExtendedKalmanEigen filter;
    InitStates states_init;
    NASReferenceValues ref_values;
    Vector3f triad_result_eul;

    uint64_t last_gps_timestamp   = 0;
    uint64_t last_accel_timestamp = 0;
    uint64_t last_gyro_timestamp  = 0;
    uint64_t last_mag_timestamp   = 0;
    uint64_t last_press_timestamp = 0;

    bool initialized = false;

    PrintLogger log = Logging::getLogger("deathstack.fsm.nas");

#ifdef DEBUG
    unsigned int counter = 0;
#endif
};

template <typename IMU, typename Press, typename GPS>
NAS<IMU, Press, GPS>::NAS(Sensor<IMU>& imu, Sensor<Press>& baro,
                          Sensor<GPS>& gps)
    : imu(imu), barometer(baro), gps(gps)
{
}

template <typename IMU, typename Press, typename GPS>
bool NAS<IMU, Press, GPS>::init()
{
    states_init.positionInit(ref_values.ref_pressure);

    states_init.velocityInit();

    Vector3f acc_init(ref_values.ref_accel_x, ref_values.ref_accel_y,
                      ref_values.ref_accel_z);
    Vector3f mag_init(ref_values.ref_mag_x, ref_values.ref_mag_y,
                      ref_values.ref_mag_z);

    triad_result_eul = states_init.triad(acc_init, mag_init);

    states_init.biasInit();

    x = states_init.getInitX();

    filter.setX(x);

    updateNASData();

#ifdef DEBUG
    Vector4f qua(x(6), x(7), x(8), x(9));
    Vector3f e = quat.quat2eul(qua);

    LOG_DEBUG(
        log,
        "Init state vector: \n px: {:.2f} \n py: {:.2f} \n pz: {:.2f} \n vx: "
        "{:.2f} \n "
        "vy: {:.2f} \n vz: {:.2f} \n roll: {:.2f} "
        "\n pitch: {:.2f} \n yaw: {:.2f}",
        x(0), x(1), x(2), x(3), x(4), x(5), e(0), e(1), e(2));

#endif

    initialized = true;

    LoggerService::getInstance()->log(getTriadResult());

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
    {
        last_accel_timestamp = imu_data.accel_timestamp;
        last_gyro_timestamp  = imu_data.gyro_timestamp;

        Vector3f accel_readings(imu_data.accel_x, imu_data.accel_y,
                                imu_data.accel_z);
        filter.predict(accel_readings);

        Vector3f gyro_readings(imu_data.gyro_x, imu_data.gyro_y,
                               imu_data.gyro_z);
        filter.predictMEKF(gyro_readings);
    }

    // check if new pressure data is available
    if (pressure_data.press_timestamp != last_press_timestamp)
    {
        last_press_timestamp = pressure_data.press_timestamp;

        filter.correctBaro(pressure_data.press);
    }

    // check if new gps data is available and the gps has fix
    if (gps_data.gps_timestamp != last_gps_timestamp && gps_data.fix == true)
    {
        last_gps_timestamp = gps_data.gps_timestamp;

        // float delta_lon = gps_data.longitude - ref_values.ref_longitude;
        // float delta_lat = gps_data.latitude - ref_values.ref_latitude;

        //Vector4f gps_readings(delta_lon, delta_lat, gps_data.velocity_north,
        //gps_data.velocity_east);

        Vector3f gps_readings(gps_data.latitude, gps_data.longitude, gps_data.height);
        Vector3f gps_ned = geodetic2NED(gps_readings);

        Vector4f pos_vel(gps_ned(0), gps_ned(1), gps_data.velocity_north, gps_data.velocity_east);
        filter.correctGPS(pos_vel, gps_data.num_satellites);
    }

    // check if new magnetometer data is available
    if (imu_data.mag_timestamp != last_mag_timestamp)
    {
        Vector3f mag_readings(imu_data.mag_x, imu_data.mag_y, imu_data.mag_z);

        if (mag_readings.norm() < EMF * JAMMING_FACTOR)
        {
            last_mag_timestamp = imu_data.mag_timestamp;

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

        /*Vector4f qua(x(6), x(7), x(8), x(9));
        Vector3f e = quat.quat2eul(qua);

        LOG_DEBUG(log,
            "State vector: \n px: {:.2f} \n py: {:.2f} \n pz: {:.2f} \n vx:
        {:.2f} \n " "vy: {:.2f} \n vz: {:.2f} \n roll: {:.2f} \n pitch: {:.2f}
        \n yaw: {:.2f} \n " "q1: {:.2f} \n q2: {:.2f} \n q3: {:.2f} \n q4 :
        {:.2f} ", x(0), x(1), x(2), x(3), x(4), x(5), e(0), e(1), e(2), x(6),
        x(7), x(8), x(9));*/
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
    Matrix<float, N, 1> state = states_init.getInitX();
    // Vector3f e = quat.quat2eul({state(6), state(7), state(8), state(9)});

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
    Vector4f q = quat.eul2quat({yaw, pitch, roll});
    x(NL)      = q(0);
    x(NL + 1)  = q(1);
    x(NL + 2)  = q(2);
    x(NL + 3)  = q(3);
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
    nas_data.timestamp = TimestampTimer::getTimestamp();

    nas_data.x = x(0);
    nas_data.y = x(1);
    nas_data.z =
        - x(2) - ref_values.ref_altitude;  // Negative sign because we're working in the NED
                         // frame but we want a positive altitude as output.
    nas_data.vx = x(3);
    nas_data.vy = x(4);
    nas_data.vz = -x(5);
    nas_data.vMod =
        sqrtf(nas_data.vx * nas_data.vx + nas_data.vy * nas_data.vy +
              nas_data.vz * nas_data.vz);
}

template <typename IMU, typename Press, typename GPS>
Vector3f NAS<IMU, Press, GPS>::geodetic2NED(const Vector3f& gps_data)
{
    const float a  = 6378137;               // [m]
    const float a2 = pow(a, 2);             // [m^2]
    const float b2 = pow(6356752.3142, 2);  // [m^2]
    const float e2 = 1 - b2 / a2;

    float lat0 = ref_values.ref_latitude * DEGREES_TO_RADIANS;
    float lon0 = ref_values.ref_longitude * DEGREES_TO_RADIANS;
    float lat  = gps_data(0) * DEGREES_TO_RADIANS;
    float lon  = gps_data(1) * DEGREES_TO_RADIANS;
    float h    = gps_data(2);

    float s1 = sin(lat0);
    float c1 = cos(lat0);
    float s2 = sin(lat);
    float c2 = cos(lat);
    float p1 = c1 * cos(lon0);
    float p2 = c2 * cos(lon);
    float q1 = c1 * sin(lon0);
    float q2 = c2 * sin(lon);
    float w1 = 1 / sqrt(1 - e2 * pow(s1, 2));
    float w2 = 1 / sqrt(1 - e2 * pow(s2, 2));
    float delta_x =
        a * (p2 * w2 - p1 * w1) + (h * p2 - ref_values.ref_altitude * p1);
    float delta_y =
        a * (q2 * w2 - q1 * w1) + (h * q2 - ref_values.ref_altitude * q1);
    float delta_z = (1 - e2) * a * (s2 * w2 - s1 * w1) +
                    (h * s2 - ref_values.ref_altitude * s1);

    // positions in ENU (east, north, up) frame
    float p_east  = -sin(lon0) * delta_x + cos(lon0) * delta_y;
    float p_north = -sin(lat0) * cos(lon0) * delta_x -
                    sin(lat0) * sin(lon0) * delta_y + cos(lat0) * delta_z;
    float p_up = cos(lat0) * cos(lon0) * delta_x +
                 cos(lat0) * sin(lon0) * delta_y + sin(lat0) * delta_z;

    // positions in NED frame
    Vector3f p_ned(p_north, p_east, -p_up);

    return p_ned;
}

}  // namespace DeathStackBoard