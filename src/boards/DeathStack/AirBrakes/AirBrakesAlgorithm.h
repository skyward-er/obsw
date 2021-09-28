/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Vincenzo Santomarco
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

#include <AirBrakes/AirBrakesData.h>
#include <AirBrakes/PID.h>
#include <AirBrakes/trajectories/Trajectory.h>
#include <Algorithm.h>
#include <LoggerService/LoggerService.h>
#include <ServoInterface.h>
#include <TimestampTimer.h>
#include <configs/AirBrakesConfig.h>
#include <diagnostic/PrintLogger.h>
#include <sensors/Sensor.h>

#include <algorithm>
#include <type_traits>

#ifdef HARDWARE_IN_THE_LOOP
#include <hardware_in_the_loop/HIL.h>
#endif

/**
 * Error and timing benchmark results:
 *
 *                         |   float        |  double
 * ==========================================================
 * Max setpoint error:     |   0.000122     |  0.000001
 * Max u error:            |   0.001045     |  0.000052
 * Max deltas error:       |   0.000000     |  0.000000
 * Max alpha error:        |   0            |  0
 * Init time:              |   0.179810 ms  |  2.037333 ms
 * Avg iter time:          |   0.151057     |  1.065261 ms
 * Total computation time: |   25.557320 ms |  181.001143 ms
 */

using namespace DeathStackBoard::AirBrakesConfigs;

namespace DeathStackBoard
{

template <class T>
class AirBrakesControlAlgorithm : public Algorithm
{

public:
    AirBrakesControlAlgorithm(Sensor<T>& sensor, ServoInterface* actuator);

    float getEstimatedCd() { return ab_data.estimated_cd; }

    bool init() override { return true; }

    /**
     * @brief This method starts the algorithm and chooses the trajectory the
     * rocket will follow for reaching apogee.
     */
    void begin();

    /**
     * @brief This method does a step if the algorithm is running, or sets the
     * actuator to 0 otherwise
     */
    void update()
    {
        if (running)
        {
            step();
        }
#ifdef HARDWARE_IN_THE_LOOP
        else
        {
            HIL::getInstance()->send(0.0);
        }
#endif
    }

    /**
     * @brief This method looks for nearest point in the current chosen
     * trajectory and sends to the servoInterface the airbrakes degree
     * according to the current rocket speed and the one in the prediction.
     */
    void step() override;

    /**
     * @brief   Computes the alpha degree (in radiants) to be sent to the
     * servoInterface in order to follow the choosen trajectory. If
     * firstIteration is true then the nearest trajectory to the current
     * rocket's coordinates is chosen.
     *
     * @param input  Input data struct, containing z, vz, vMod and timestamp.
     * @param firstIteration    The flag indicating wheter the trajectory has
     * already been choosen or not.
     *
     * @returns The alpha in radiants to be sent to the servoInterface
     */
    float computeAlpha(T input, bool firstIteration);

    /**
     * @brief Chooses the nearest point on the trajectory, considering all the
     * trajectories. Given the rocket initial input, stores the nearest
     * trajectory.
     *
     * @param z     The altitude of the rocket
     * @param vz    The z component of the rocket's speed
     */
    TrajectoryPoint chooseTrajectory(float z, float vz);

    /**
     * @brief  Starting from the second iteration selects the nearest point on
     * the choosen trajectory. This method does not consider the Euclidean
     * distance, but only the altitude (z) distance.
     *
     * @param z     The altitude of the rocket
     * @param vz    The z component of the rocket's speed
     */
    TrajectoryPoint getSetpoint(float z, float vz);

    /**
     * @brief Update Pid internal input. The Pid output represents represents
     * the drag force of the wind.
     *
     * @param z         Current rocket's altitude
     * @param vz        The z component of the rocket's speed
     * @param vMod     Modulo of the rocket's speed
     * @param rho       The density of the air
     * @param setpoint  Nearest point to the current rocket input in the chosen
     * trajectory
     */
    float pidStep(float z, float vz, float vMod, float rho,
                  TrajectoryPoint setpoint);

    /**
     * @brief Compute the necessary airbrakes surface to match the
     * given the drag force from the Pid. The possible surface values are
     * discretized in (S_MIN, S_MAX) with a
     * step of S_STEP. For every possible deltaS the
     * correspondig drag force is computed with @see getDrag method and the one
     * that gives lowest error with respect to Pid output is returned.
     *
     * @param z     The current altitude of the rocket
     * @param vz    The z component of the rocket's speed
     * @param vMod  Modulo of the rocket's speed
     * @param rho   The density of the air
     * @param u     The current pid value
     */
    float getSurface(float z, float vz, float vMod, float rho, float u);

    /**
     * @brief Maps the exposed airbrakes surface to servoInterface angle in
     * degrees.
     *
     * @param s    Airbrakes surface
     */
    float getAlpha(float s);

    /**
     * @param h     The current altitude
     * @return The density of air according to current altitude.
     */
    float getRho(float h);

    /**
     * @param vMod    Modulo of the current rocket speed.
     * @param z        The z component of the rocket's speed
     * @return The speed in Mach unit.
     */
    float getMach(float vMod, float z);

    /**
     * @param s    Airbrakes surface
     * @return The radial distance of the current airbrakes extension.
     */
    float getExtension(float s);

    /**
     * @param v   Rocket's velocity module
     * @param h   The current altitude of the rocket
     * @param s   Airbrakes surface
     * @return The drag force coefficient
     */
    float getDrag(float v, float h, float s);

    /**
     * @brief Log algorithm data structure
     *
     */
    void logAlgorithmData(T input, uint64_t t);

    /**
     * @brief Log airbrakes data structure
     */
    void logAirbrakesData(uint64_t t);

private:
    int indexMinVal = 0;
    float alpha     = 0;
    uint64_t ts     = 0;
    Trajectory chosenTrajectory;
    ServoInterface* actuator;
    Sensor<T>& sensor;
    PIController pid;

    uint64_t begin_ts = 0;

    AirBrakesAlgorithmData algo_data;
    AirBrakesData ab_data;

    LoggerService& logger;

    std::is_same<float, decltype(std::declval<T>().z)> checkz;
    std::is_same<float, decltype(std::declval<T>().vz)> checkvz;
    std::is_same<float, decltype(std::declval<T>().vMod)> checkvMod;
    std::is_same<uint64_t, decltype(std::declval<T>().timestamp)>
        checktimestamp;
};

template <class T>
AirBrakesControlAlgorithm<T>::AirBrakesControlAlgorithm(
    Sensor<T>& sensor, ServoInterface* actuator)
    : actuator(actuator), sensor(sensor), pid(Kp, Ki),
      logger(*(LoggerService::getInstance()))
{
}

template <class T>
void AirBrakesControlAlgorithm<T>::begin()
{
    if (running)
    {
        return;
    }

    running = true;

    begin_ts = TimestampTimer::getTimestamp();

    ts = (sensor.getLastSample()).timestamp;

    alpha = computeAlpha(sensor.getLastSample(), true);

#ifndef ROCCARASO
    actuator->set(alpha, true);
#endif
}

template <class T>
void AirBrakesControlAlgorithm<T>::step()
{
    T input = sensor.getLastSample();

    if (input.timestamp > ts)
    {
        ts    = input.timestamp;
        alpha = computeAlpha(input, false);
    }

#ifndef ROCCARASO
    actuator->set(alpha, true);
#endif

    uint64_t t = TimestampTimer::getTimestamp();
    logAlgorithmData(input, t);
    logAirbrakesData(t);

#ifdef ROCCARASO
    if (t - begin_ts < AB_OPENING_TIMEOUT * 1000)
    {  // after 3 seconds open to 100%
        actuator->set(AB_SERVO_MAX_POS, true);
    }
    else if (t - begin_ts > AB_OPENING_TIMEOUT * 1000 &&
             t - begin_ts < 2 * AB_OPENING_TIMEOUT * 1000)
    {  // after 6 seconds open to 100%
        actuator->set(AB_SERVO_MAX_POS / 2, true);
    }
    else
    {  // then keep airbrakes closed
        actuator->set(AB_SERVO_MIN_POS, true);
    }
#endif
}

template <class T>
float AirBrakesControlAlgorithm<T>::computeAlpha(T input, bool firstIteration)
{
    float z    = input.z;
    float vz   = input.vz;
    float vMod = input.vMod;
    float rho  = getRho(z);

    TrajectoryPoint setpoint;

    if (firstIteration)
    {
        setpoint = chooseTrajectory(z, vz);
    }
    else
    {
        setpoint = getSetpoint(z, vz);
    }

    float u     = pidStep(z, vz, vMod, rho, setpoint);
    float s     = getSurface(z, vz, vMod, rho, u);
    float alpha = getAlpha(s);

    // for logging
    ab_data.rho     = rho;
    ab_data.u       = u;
    ab_data.surface = s;

    return alpha;
}

template <class T>
TrajectoryPoint AirBrakesControlAlgorithm<T>::chooseTrajectory(float z,
                                                               float vz)
{
    TrajectoryPoint currentPoint(z, vz);

    float bestMin = INFINITY;

    for (uint8_t trajectoryIndex = 0; trajectoryIndex < TOT_TRAJECTORIES;
         trajectoryIndex++)
    {
        Trajectory trajectory(trajectoryIndex, S_MAX);

        for (uint32_t pointIndex = 0; pointIndex < trajectory.length();
             pointIndex++)
        {
            TrajectoryPoint ref = trajectory.get(pointIndex);
            float distanceFromCurrentinput =
                TrajectoryPoint::distance(ref, currentPoint);

            if (distanceFromCurrentinput < bestMin)
            {
                bestMin          = distanceFromCurrentinput;
                indexMinVal      = pointIndex;
                chosenTrajectory = trajectory;
            }
        }
    }

    logger.log(
        AirBrakesChosenTrajectory{chosenTrajectory.getTrajectoryIndex()});

    PrintLogger log = Logging::getLogger("deathstack.fsm.abk");
    LOG_INFO(log, "Chosen trajectory : {:d} \n",
             chosenTrajectory.getTrajectoryIndex());

    TrajectoryPoint setpoint = chosenTrajectory.get(indexMinVal);

    return setpoint;
}

template <class T>
TrajectoryPoint AirBrakesControlAlgorithm<T>::getSetpoint(float z, float vz)
{
    TrajectoryPoint currentPoint(z, vz);
    float minDistance = INFINITY;

    uint32_t start = std::max(indexMinVal + START_INDEX_OFFSET, 0);
    uint32_t end   = chosenTrajectory.length();

    for (uint32_t pointIndex = start; pointIndex < end; pointIndex++)
    {
        TrajectoryPoint ref = chosenTrajectory.get(pointIndex);
        float distanceFromCurrentinput =
            TrajectoryPoint::distance(ref, currentPoint);
        if (distanceFromCurrentinput < minDistance)
        {
            minDistance = distanceFromCurrentinput;
            indexMinVal = pointIndex;
        }
    }

    TrajectoryPoint setpoint = chosenTrajectory.get(indexMinVal);

    // for logging
    ab_data.setpoint_z  = setpoint.getZ();
    ab_data.setpoint_vz = setpoint.getVz();

    return setpoint;
}

template <class T>
float AirBrakesControlAlgorithm<T>::pidStep(float z, float vz, float vMod,
                                            float rho, TrajectoryPoint setpoint)
{
    // cd minimum if abk surface is 0
    float cd_min = getDrag(vMod, z, 0);
    // cd maximum if abk surface is maximum
    float cd_max = getDrag(vMod, z, S_MAX);

    float u_min = 0.5 * rho * cd_min * S0 * vz * vMod;
    float u_max = 0.5 * rho * cd_max * S0 * vz * vMod;

    // get reference CD and control action, according to the chosen trajectory
    float cd_ref = getDrag(vMod, z, chosenTrajectory.getRefSurface());
    float u_ref  = 0.5 * rho * cd_ref * S0 * vz * vMod;

    float error       = vz - setpoint.getVz();
    ab_data.pid_error = error; // for logging

    // update PI controller
    float u = pid.update(error);
    u       = u + u_ref;
    u       = pid.antiWindUp(u, u_min, u_max);

    return u;
}

template <class T>
float AirBrakesControlAlgorithm<T>::getSurface(float z, float vz, float vMod,
                                               float rho, float u)
{
    float estimated_cd = 0;
    float selected_s   = 0;
    float best_du      = INFINITY;

    for (float s = S_MIN; s < S_MAX + S_STEP; s += S_STEP)
    {
        float cd = getDrag(vMod, z, s);
        float du = abs(u - (0.5 * rho * S0 * cd * vz * vMod));

        if (du < best_du)
        {
            best_du      = du;
            selected_s   = s;
            estimated_cd = cd;
        }
    }

    ab_data.estimated_cd = estimated_cd; // for logging

    return selected_s;
}

template <class T>
float AirBrakesControlAlgorithm<T>::getAlpha(float s)
{
    float alpha_rad =
        (-B_DELTAS + sqrt(powf(B_DELTAS, 2) + 4 * A_DELTAS * s)) /
        (2 * A_DELTAS);

    float alpha_deg = alpha_rad * 180.0f / PI;

    return alpha_deg;
}

template <class T>
float AirBrakesControlAlgorithm<T>::getRho(float h)
{
    return RHO * expf(-h / Hn);
}

template <class T>
float AirBrakesControlAlgorithm<T>::getMach(float vMod, float z)
{
    float c = Co + ALPHA * z;
    return vMod / c;
}

template <class T>
float AirBrakesControlAlgorithm<T>::getExtension(float s)
{
    return (-B + sqrtf(powf(B, 2) + 4 * A * s)) / (2 * A);
}

template <class T>
float AirBrakesControlAlgorithm<T>::getDrag(float v, float h, float s)
{
    float x = getExtension(s);

    float mach = getMach(v, h);

    float pow_mach[7] = {1,
                        mach,
                        powf(mach, 2),
                        powf(mach, 3),
                        powf(mach, 4),
                        powf(mach, 5),
                        powf(mach, 6)};

    return coeffs.n000 + coeffs.n100 * pow_mach[1] + coeffs.n200 * pow_mach[2] +
           coeffs.n300 * pow_mach[3] + coeffs.n400 * pow_mach[4] +
           coeffs.n500 * pow_mach[5] + coeffs.n600 * pow_mach[6] +
           coeffs.n010 * x + coeffs.n020 * powf(x, 2) +
           coeffs.n110 * x * pow_mach[1] +
           coeffs.n120 * powf(x, 2) * pow_mach[1] +
           coeffs.n210 * x * pow_mach[2] +
           coeffs.n220 * powf(x, 2) * pow_mach[2] +
           coeffs.n310 * x * pow_mach[3] +
           coeffs.n320 * powf(x, 2) * pow_mach[3] +
           coeffs.n410 * x * pow_mach[4] +
           coeffs.n420 * powf(x, 2) * pow_mach[4] +
           coeffs.n510 * x * pow_mach[5] +
           coeffs.n520 * powf(x, 2) * pow_mach[5] + coeffs.n001 * h;
}

template <class T>
void AirBrakesControlAlgorithm<T>::logAlgorithmData(T input, uint64_t t)
{
    AirBrakesAlgorithmData d;
    d.timestamp = t;
    d.z         = input.z;
    d.vz        = input.vz;
    d.vMod      = input.vMod;
    logger.log(d);
}

template <class T>
void AirBrakesControlAlgorithm<T>::logAirbrakesData(uint64_t t)
{
    ab_data.timestamp      = t;
    ab_data.servo_position = actuator->getCurrentPosition();
    // estimated_cd set when computing the new alpha (in getDrag())
    // pid_error inserted when computing the new alpha (in pidStep())
    // setpoint z and vz inserted when computing the new setpoint (in
    // getSetpoint())
    // the same holds to u, surface and rho, in computeAlpha()
    logger.log(ab_data);
}

}  // namespace DeathStackBoard
