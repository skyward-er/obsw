/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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

#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Configs/WingConfig.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/Wing/WindEstimation.h>
#include <common/events/Events.h>

using namespace Parafoil::WingConfig;
using namespace Boardcore;
using namespace Common;
namespace Parafoil
{

WindEstimation::WindEstimation()
{
    running    = false;
    calRunning = false;
    funv << 1.0f, 0.0f, 0.0f, 1.0f;  // cppcheck-suppress constStatement
}

WindEstimation::~WindEstimation()
{
    stopWindEstimationSchemeCalibration();
    stoptWindEstimationScheme();
}

void WindEstimation::startWindEstimationSchemeCalibration()
{
    if (!calRunning)
    {
        calRunning = true;
        nSampleCal = 0;
        vx         = 0;
        vy         = 0;
        v2         = 0;
        // Register the task
        BoardScheduler::getInstance().getScheduler().addTask(
            std::bind(&WindEstimation::WindEstimationSchemeCalibration, this),
            WIND_CALIBRATION_UPDATE_PERIOD, WIND_CALIBRATION_ID);
        LOG_INFO(logger, "WindEstimationCalibration started");
    }
}

void WindEstimation::stopWindEstimationSchemeCalibration()
{
    if (calRunning)
    {
        calRunning = false;
        BoardScheduler::getInstance().getScheduler().removeTask(
            WIND_CALIBRATION_ID);
        LOG_INFO(logger, "WindEstimationSchemeCalibration stopped");
        // assign value to the array only if running!= false;
        if (!running)
        {
            miosix::Lock<FastMutex> l(mutex);
            wind          = windCalibration;
            windLogger.vx = windCalibration[0];
            windLogger.vy = windCalibration[1];
            logStatus();
        }
    }
}

void WindEstimation::WindEstimationSchemeCalibration()
{
    if (nSampleCal < WIND_CALIBRATION_SAMPLE_NUMBER)
    {
        auto gpsData = Sensors::getInstance().getUbxGpsLastSample();
        calibrationMatrix(nSampleCal, 0) = gpsData.velocityNorth;
        calibrationMatrix(nSampleCal, 1) = gpsData.velocityEast;
        calibrationV2(nSampleCal) =
            gpsData.velocityNorth * gpsData.velocityNorth +
            gpsData.velocityEast * gpsData.velocityEast;
        vx += calibrationMatrix(nSampleCal, 0);
        vy += calibrationMatrix(nSampleCal, 1);
        v2 += calibrationV2(nSampleCal);
        nSampleCal++;
    }
    else
    {
        vx = vx / nSampleCal;
        vy = vy / nSampleCal;
        v2 = v2 / nSampleCal;
        for (int i = 0; i < nSampleCal; i++)
        {
            calibrationMatrix(i, 0) = calibrationMatrix(i, 0) - vx;
            calibrationMatrix(i, 1) = calibrationMatrix(i, 1) - vy;
            calibrationV2(i)        = 0.5f * (calibrationV2(i) - v2);
        }

        // Eigen::Matrix<float, 2, WIND_CALIBRATION_SAMPLE_NUMBER>
        Eigen::MatrixXf calibrationMatrixT = calibrationMatrix.transpose();
        windCalibration = (calibrationMatrixT * calibrationMatrix)
                              .ldlt()
                              .solve(calibrationMatrixT * calibrationV2);
        EventBroker::getInstance().post(FLIGHT_WIND_PREDICTION, TOPIC_FLIGHT);
    }
}

void WindEstimation::startWindEstimationScheme()
{
    if (!running)
    {
        running = true;
        //  Register the task
        nSample = nSampleCal;
        BoardScheduler::getInstance().getScheduler().addTask(
            std::bind(&WindEstimation::WindEstimationScheme, this),
            WIND_PREDICTION_UPDATE_PERIOD, WING_PREDICTION_ID);
        LOG_INFO(logger, "WindEstimationScheme started");
    }
}

void WindEstimation::stoptWindEstimationScheme()
{
    if (running)
    {
        running = false;
        BoardScheduler::getInstance().getScheduler().removeTask(
            WING_PREDICTION_ID);
        LOG_INFO(logger, "WindEstimationScheme ended");
    }
}

void WindEstimation::WindEstimationScheme()
{
    Eigen::Vector2f phi;
    Eigen::Matrix<float, 1, 2> phiT;
    Eigen::Vector2f temp;
    float y;

    auto gpsData = Sensors::getInstance().getUbxGpsLastSample();
    // update avg
    vx = (wind(0) * nSample + gpsData.velocityNorth) / (nSample + 1);
    vy = (wind(1) * nSample + gpsData.velocityEast) / (nSample + 1);
    v2 = (v2 * nSample + (vx * vx + vy * vy)) / (nSample + 1);
    nSample++;
    phi(0) = gpsData.velocityNorth - vx;
    phi(1) = gpsData.velocityEast - vy;
    y      = 0.5f * ((vx * vx + vy * vy) - v2);

    phiT = phi.transpose();
    funv = (funv - (funv * phi * phiT * funv) / (1 + (phiT * funv * phi)));
    temp = (0.5 * (funv + funv.transpose()) * phi) *
           (y - phiT * wind);  // maybe moved to mutex ReadOnly
    {
        miosix::Lock<FastMutex> l(mutex);
        wind          = wind + temp;
        windLogger.vx = wind[0];
        windLogger.vy = wind[1];
        logStatus();
    }
}

Eigen::Vector2f WindEstimation::getWindEstimationScheme()
{
    miosix::PauseKernelLock l;
    return wind;
}

void WindEstimation::logStatus()
{
    windLogger.timestamp = TimestampTimer::getTimestamp();
    Logger::getInstance().log(
        windLogger);  // ask wether this has to be removed from the mutex or not
}

}  // namespace Parafoil
