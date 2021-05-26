/*
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Vincenzo Santomarco
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

#include <algorithm>
#include <type_traits>

#include "../Algorithm.h"
#include "../ServoInterface.h"
#include "AeroBrakesData.h"
#include "Pid.h"
#include "TimestampTimer.h"
#include "configs/AeroBrakesConfig.h"
#include "sensors/Sensor.h"
#include "trajectories/Trajectory.h"
#include "LoggerService/LoggerService.h"

/*
    Error and timing benchmark results:

                            |   float        |  double
    ==========================================================
    Max setpoint error:     |   0.000122     |  0.000001
    Max u error:            |   0.001045     |  0.000052
    Max deltas error:       |   0.000000     |  0.000000
    Max alpha error:        |   0            |  0
    Init time:              |   0.179810 ms  |  2.037333 ms
    Avg iter time:          |   0.151057     |  1.065261 ms
    Total computation time: |   25.557320 ms |  181.001143 ms

*/

using namespace DeathStackBoard::AeroBrakesConfigs;

namespace DeathStackBoard
{

template <class T>
class AeroBrakesControlAlgorithm : public Algorithm
{
private:
    int indexMinVal = 0;
    float alpha     = 0;
    uint64_t ts     = 0;
    Trajectory chosenTrajectory;
    ServoInterface* actuator;
    Sensor<T>& sensor;
    Pid pid;

    LoggerService& logger;

    std::is_same<float, decltype(std::declval<T>().z)> checkz;
    std::is_same<float, decltype(std::declval<T>().vz)> checkvz;
    std::is_same<float, decltype(std::declval<T>().vMod)> checkvMod;
    std::is_same<uint64_t, decltype(std::declval<T>().timestamp)>
        checktimestamp;

public:
    AeroBrakesControlAlgorithm(Sensor<T>& sensor, ServoInterface* actuator);

    bool init() override { return true; }

    /**
     * @brief This method starts the algorithm and chooses the trajectory the
     * rocket will follow for reaching apogee.
     * */
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
        else
        {
            actuator->reset();
        }
    }

    /**
     * @brief This method looks for nearest point in the current chosen
     * trajectory and sends to the servoInterface the aerobrakes degree
     * according to the current rocket speed and the one in the prediction.
     * */
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
     * */
    float computeAlpha(T input, bool firstIteration);

    /**
     * @brief Chooses the nearest point on the trajectory, considering all the
     * trajectories. Given the rocket initial input, stores the nearest
     * trajectory.
     *
     * @param z     The altitude of the rocket
     * @param vz    The z component of the rocket's speed
     * */
    TrajectoryPoint chooseTrajectory(float z, float vz);

    /**
     * @brief  Starting from the second iteration selects the nearest point on
     * the choosen trajectory. This method does not consider the Euclidean
     * distance, but only the altitude (z) distance.
     *
     * @param z     The altitude of the rocket
     * @param vz    The z component of the rocket's speed
     * */
    TrajectoryPoint getSetpoint(float z, float vz);

    /**
     * @brief Update Pid internal input. The Pid output represents represents
     * the drag force of the wind.
     *
     * @param vz        The z component of the rocket's speed
     * @param vMod      Modulo of the rocket's speed
     * @param rho       The density of the air
     * @param setpoint  Nearest point to the current rocket input in the chosen
     * trajectory
     * */
    float pidStep(float vz, float vMod, float rho, TrajectoryPoint setpoint);

    /**
     * @brief Compute the necessary aerobrakes surface to match the
     *    given the drag force from the Pid. The possible deltaS values are
     *    discretized in (DELTA_S_AVAILABLE_MIN, DELTA_S_AVAILABLE_MAX) with a
     * step of DELTA_S_AVAILABLE_STEP. For every possible deltaS the
     * correspondig drag force is computed with @see getDrag method and the one
     * that gives lowest error with respect to Pid output is returned.
     *
     * @param z     The current altitude of the rocket
     * @param vz    The z component of the rocket's speed
     * @param vMod  Modulo of the rocket's speed
     * @param rho   The density of the air
     * @param u     The current pid value
     * */
    float getDeltaS(float z, float vz, float vMod, float rho, float u);

    /**
     * @brief Maps the exposed aerobrakes surface to servoInterface angle in
     * radiants.
     *
     * @param deltas    Total variation of the aerobrakes surface
     * */
    float getAlpha(float deltaS);

    /**
     * @param h     The current altitude
     * @return The density of air according to current altitude.
     * */
    float getRho(float h);

    /**
     * @param v_mod    Modulo of the current rocket speed.
     * @param z        The z component of the rocket's speed
     * @return The speed in Mach unit.
     * */
    float getMach(float v_mod, float z);

    /**
     * @param deltaS    The total variation of the aerobrakes surface
     * @return The radial distance of the current aerobrakes extension.
     * */
    float getExtension(float deltaS);

    /**
     * @param h         The current altitude of the rocket
     * @param s
     * @param powMach   Array containing precomputed powers of the mach
     * @return The drag force coefficient
     * */
    float getDrag(float h, float s, float* powMach);

    /**
     * @brief Log algorithm data structure
     *
     */
    void logData(T input);
};

template <class T>
AeroBrakesControlAlgorithm<T>::AeroBrakesControlAlgorithm(
    Sensor<T>& sensor, ServoInterface* actuator)
    : actuator(actuator), sensor(sensor), pid(Kp, Ki),
      logger(*(LoggerService::getInstance()))
{
}

template <class T>
void AeroBrakesControlAlgorithm<T>::begin()
{
    if (running)
    {
        return;
    }

    running = true;

    ts = (sensor.getLastSample()).timestamp;

    alpha = computeAlpha(sensor.getLastSample(), true);
    actuator->set(alpha, true);
}

template <class T>
void AeroBrakesControlAlgorithm<T>::step()
{
    T input = sensor.getLastSample();

    if (input.timestamp > ts)
    {
        ts    = input.timestamp;
        alpha = computeAlpha(input, false);
    }

    actuator->set(alpha, true);

    logData(input);
}

template <class T>
float AeroBrakesControlAlgorithm<T>::computeAlpha(T input, bool firstIteration)
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

    float u      = pidStep(vz, vMod, rho, setpoint);
    float deltaS = getDeltaS(z, vz, vMod, rho, u);
    float alpha  = getAlpha(deltaS);

    return alpha;
}

template <class T>
TrajectoryPoint AeroBrakesControlAlgorithm<T>::chooseTrajectory(float z,
                                                                float vz)
{
    TrajectoryPoint currentPoint(z, vz);

    float bestMin = INFINITY;

    for (uint8_t trajectoryIndex = 0; trajectoryIndex < TOT_TRAJECTORIES;
         trajectoryIndex++)
    {

        Trajectory trajectory(trajectoryIndex, DELTA_S_AVAILABLE_MAX);

        for (int pointIndex = 0; pointIndex < LOOKS; pointIndex++)
        {
            TrajectoryPoint ref = trajectory.get(pointIndex);
            float distancesFromCurrentinput =
                TrajectoryPoint::distance(ref, currentPoint);

            if (distancesFromCurrentinput < bestMin)
            {
                bestMin          = distancesFromCurrentinput;
                indexMinVal      = pointIndex;
                chosenTrajectory = trajectory;
            }
        }
    }

    logger.log(
        AeroBrakesChosenTrajectory{chosenTrajectory.getTrajectoryIndex()});

    TRACE("[AeroBrakes] Chosen trajectory : %d \n",
          chosenTrajectory.getTrajectoryIndex());

    TrajectoryPoint setpoint = chosenTrajectory.get(indexMinVal);

    return setpoint;
}

template <class T>
TrajectoryPoint AeroBrakesControlAlgorithm<T>::getSetpoint(float z, float vz)
{
    TrajectoryPoint currentPoint(z, vz);
    float minDistance = INFINITY;

    int start = std::max(indexMinVal + START_INDEX_OFFSET, 0);
    int end   = chosenTrajectory.length();

    for (int pointIndex = start; pointIndex < end; pointIndex++)
    {
        TrajectoryPoint ref = chosenTrajectory.get(pointIndex);
        float DistanceFromCurrentinput =
            TrajectoryPoint::zDistance(ref, currentPoint);
        if (DistanceFromCurrentinput < minDistance)
        {
            minDistance = DistanceFromCurrentinput;
            indexMinVal = pointIndex;
        }
    }

    TrajectoryPoint setpoint = chosenTrajectory.get(indexMinVal);

    return setpoint;
}

template <class T>
float AeroBrakesControlAlgorithm<T>::pidStep(float vz, float vMod, float rho,
                                             TrajectoryPoint setpoint)
{
    float umin = 0;
    // The *1 factor replaces the cd variable. This variable will be later
    // calculated in the getDeltaS method.
    float umax  = 0.5 * rho * S0 * 1 * vz * vMod;
    float error = (vz - setpoint.getVz());

    float u = pid.step(umin, umax, error);

    return u;
}

template <class T>
float AeroBrakesControlAlgorithm<T>::getDeltaS(float z, float vz, float vMod,
                                               float rho, float u)
{
    float deltaS    = 0;
    float bestValue = INFINITY;

    float mach = getMach(vMod, z);
    // Precomputes all the powers of mach (from 0 to 6) in order to reuse it
    // at avery iteration of the deltaSAvailable loop.
    // See getDrag method.
    float powMach[7] = {1,
                        mach,
                        powf(mach, 2),
                        powf(mach, 3),
                        powf(mach, 4),
                        powf(mach, 5),
                        powf(mach, 6)};

    for (float deltaSAvailable = DELTA_S_AVAILABLE_MIN;
         deltaSAvailable < DELTA_S_AVAILABLE_MAX + DELTA_S_AVAILABLE_STEP;
         deltaSAvailable += DELTA_S_AVAILABLE_STEP)
    {
        float cdAvaliable = getDrag(z, deltaSAvailable, powMach);
        float temp        = abs(u - 0.5 * rho * S0 * cdAvaliable * vz * vMod);
        if (temp < bestValue)
        {
            bestValue = temp;
            deltaS    = deltaSAvailable;
        }
    }

    return deltaS;
}

template <class T>
float AeroBrakesControlAlgorithm<T>::getAlpha(float deltaS)
{
    float alphaRadiants =
        (-B_DELTAS + sqrt(powf(B_DELTAS, 2) + 4 * A_DELTAS * deltaS)) /
        (2 * A_DELTAS);
    float alphaDegrees = alphaRadiants * 180.0f / PI;

    return alphaDegrees;
}

template <class T>
float AeroBrakesControlAlgorithm<T>::getRho(float h)
{
    return RHO * expf(-h / Hn);
}

template <class T>
float AeroBrakesControlAlgorithm<T>::getMach(float v_mod, float z)
{
    float c = Co + ALPHA * z;
    return v_mod / c;
}

template <class T>
float AeroBrakesControlAlgorithm<T>::getExtension(float deltaS)
{
    return (-B + sqrtf(powf(B, 2) + 4 * A * deltaS)) / (2 * A);
}

template <class T>
float AeroBrakesControlAlgorithm<T>::getDrag(float h, float s, float* powMach)
{
    float x = getExtension(s);

    return coeffs.n000 + coeffs.n100 * powMach[1] + coeffs.n200 * powMach[2] +
           coeffs.n300 * powMach[3] + coeffs.n400 * powMach[4] +
           coeffs.n500 * powMach[5] + coeffs.n600 * powMach[6] +
           coeffs.n010 * x + coeffs.n020 * powf(x, 2) +
           coeffs.n110 * x * powMach[1] +
           coeffs.n120 * powf(x, 2) * powMach[1] +
           coeffs.n210 * x * powMach[2] +
           coeffs.n220 * powf(x, 2) * powMach[2] +
           coeffs.n310 * x * powMach[3] +
           coeffs.n320 * powf(x, 2) * powMach[3] +
           coeffs.n410 * x * powMach[4] +
           coeffs.n420 * powf(x, 2) * powMach[4] +
           coeffs.n510 * x * powMach[5] +
           coeffs.n520 * powf(x, 2) * powMach[5] + coeffs.n001 * h;
}

template <class T>
void AeroBrakesControlAlgorithm<T>::logData(T input)
{
    AeroBrakesAlgorithmData d;
    d.timestamp = input.timestamp;
    d.z = input.z;
    d.vz = input.vz;
    d.vMod = input.vMod;
    logger.log(d);
}

}  // namespace DeathStackBoard
