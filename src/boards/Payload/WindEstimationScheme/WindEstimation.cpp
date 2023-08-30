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

#include "WindEstimation.h"

#include <Payload/BoardScheduler.h>
#include <Payload/Configs/WESConfig.h>
#include <Payload/Sensors/Sensors.h>
#include <common/Events.h>
#include <events/EventBroker.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace Payload::WESConfig;
using namespace Boardcore;
using namespace Common;
using namespace miosix;

namespace Payload
{

WindEstimation::WindEstimation(TaskScheduler* sched)
    : scheduler(sched), running(false), calRunning(false)
{
    funv << 1.0f, 0.0f, 0.0f, 1.0f;  // cppcheck-suppress constStatement
}

bool WindEstimation::start()
{
    scheduler->addTask(
        std::bind(&WindEstimation::windEstimationSchemeCalibration, this),
        WES_CALIBRATION_UPDATE_PERIOD);

    // Register the WES task
    scheduler->addTask(std::bind(&WindEstimation::windEstimationScheme, this),
                       WES_PREDICTION_UPDATE_PERIOD);

    return true;
}

WindEstimation::~WindEstimation()
{
    stopWindEstimationSchemeCalibration();
    stopWindEstimationScheme();
}

bool WindEstimation::getStatus() { return running; }

void WindEstimation::startWindEstimationSchemeCalibration()
{
    if (!calRunning && !running)
    {
        calRunning = true;
        nSampleCal = 0;
        vx         = 0;
        vy         = 0;
        v2         = 0;
        LOG_INFO(logger, "WindEstimationCalibration started");
    }
}

void WindEstimation::stopWindEstimationSchemeCalibration()
{
    calRunning = false;
    LOG_INFO(logger, "WindEstimationSchemeCalibration stopped");
    // assign value to the array only if running != false;
    if (!running)
    {
        miosix::Lock<FastMutex> l(mutex);
        wind = windCalibration;
    }
    windLogger.vn = windCalibration[0];
    windLogger.ve = windCalibration[1];
    logStatus();
}

void WindEstimation::windEstimationSchemeCalibration()
{

    if (calRunning)
    {
        auto gpsData =
            ModuleManager::getInstance().get<Sensors>()->getGPSLastSample();
        if (gpsData.fix != 0)
        {
            if (nSampleCal < WES_CALIBRATION_SAMPLE_NUMBER)
            {
                float gpsN = 0, gpsE = 0;

                gpsN                             = gpsData.velocityNorth;
                gpsE                             = gpsData.velocityEast;
                calibrationMatrix(nSampleCal, 0) = gpsN;
                calibrationMatrix(nSampleCal, 1) = gpsE;
                calibrationV2(nSampleCal)        = gpsN * gpsN + gpsE * gpsE;

                vx += gpsN;
                vy += gpsE;
                v2 += gpsN * gpsN + gpsE * gpsE;
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
                Eigen::MatrixXf calibrationMatrixT =
                    calibrationMatrix.transpose();
                windCalibration =
                    (calibrationMatrixT * calibrationMatrix)
                        .ldlt()
                        .solve(calibrationMatrixT * calibrationV2);
                stopWindEstimationSchemeCalibration();
                startWindEstimationScheme();
            }
        }
    }
}

void WindEstimation::startWindEstimationScheme()
{
    if (!running)
    {
        running = true;
        nSample = nSampleCal;  // TODO explicit that nSample has to be reset
                               // only the first time startWES is called
        LOG_INFO(logger, "WindEstimationScheme started");
    }
}

void WindEstimation::stopWindEstimationScheme()
{
    if (running)
    {
        running = false;
        LOG_INFO(logger, "WindEstimationScheme ended");
    }
}

void WindEstimation::windEstimationScheme()
{
    if (running)
    {
        auto gpsData =
            ModuleManager::getInstance().get<Sensors>()->getGPSLastSample();
        if (gpsData.fix != 0)
        {
            Eigen::Vector2f phi;
            Eigen::Matrix<float, 1, 2> phiT;
            Eigen::Vector2f temp;
            float y, gpsN = 0, gpsE = 0;
            gpsN = gpsData.velocityNorth;
            gpsE = gpsData.velocityEast;
            // update avg
            nSample++;
            vx = (vx * nSample + gpsN) / (nSample + 1);
            vy = (vy * nSample + gpsE) / (nSample + 1);
            v2 = (v2 * nSample + (gpsN * gpsN + gpsE * gpsE)) / (nSample + 1);
            phi(0) = gpsN - vx;
            phi(1) = gpsE - vy;
            y      = 0.5f * ((gpsN * gpsN + gpsE * gpsE) - v2);

            phiT = phi.transpose();
            funv =
                (funv - (funv * phi * phiT * funv) / (1 + (phiT * funv * phi)));
            temp = (0.5 * (funv + funv.transpose()) * phi) *
                   (y - phiT * getWindEstimationScheme());
            {
                miosix::Lock<FastMutex> l(mutex);
                wind          = wind + temp;
                windLogger.vn = wind[0];
                windLogger.ve = wind[1];
            }
            logStatus();
        }
    }
}

Eigen::Vector2f WindEstimation::getWindEstimationScheme()
{
    miosix::Lock<FastMutex> l(mutex);
    return wind;
}

void WindEstimation::logStatus()
{
    windLogger.timestamp = TimestampTimer::getTimestamp();
    Logger::getInstance().log(windLogger);
}

}  // namespace Payload
