/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Federico Mandelli, Davide Basso
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

#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Configs/WESConfig.h>
#include <Parafoil/Sensors/Sensors.h>

using namespace Boardcore;
using namespace Boardcore::Units::Speed;
using namespace Parafoil::Config;

namespace Parafoil
{
WindEstimation::WindEstimation()
{
    funv << 1.0f, 0.0f, 0.0f, 1.0f;  // cppcheck-suppress constStatement
}

WindEstimation::~WindEstimation()
{
    stopAlgorithm();
    stopCalibration();
}

bool WindEstimation::start()
{
    auto& scheduler = getModule<BoardScheduler>()->windEstimation();

    auto calibrationTask = scheduler.addTask([this] { updateCalibration(); },
                                             WES::CALIBRATION_UPDATE_PERIOD);

    if (calibrationTask == 0)
    {
        LOG_ERR(logger, "Failed to start wind estimation calibration task");
        return false;
    }

    auto algorithmTask = scheduler.addTask([this] { updateAlgorithm(); },
                                           WES::PREDICTION_UPDATE_PERIOD);

    if (algorithmTask == 0)
    {
        LOG_ERR(logger, "Failed to start wind estimation algorithm task");
        return false;
    }

    started = true;
    return true;
}

bool WindEstimation::isStarted() { return started; }

void WindEstimation::startCalibration()
{
    if (!calibrating)
    {
        calibrating        = true;
        nSampleCalibration = 0;

        velocity.vn = 0_mps;
        velocity.ve = 0_mps;

        speedSquared = 0;

        LOG_INFO(logger, "Wind estimation calibration started");
    }
    else
    {
        LOG_WARN(logger, "Wind estimation calibration already started");
    }
}

void WindEstimation::stopCalibration()
{
    if (calibrating)
    {
        LOG_INFO(logger, "Wind estimation calibration stopped");

        if (!running)
        {
            miosix::Lock<FastMutex> l(mutex);
            wind = calibratingWind;
        }
        else
        {
            LOG_WARN(logger,
                     "Wind estimation algorithm is running, calibration wind "
                     "will not be applied");
        }

        logStatus();
        calibrating = false;
    }
    else
    {
        LOG_WARN(logger, "Wind estimation calibration already stopped");
    }
}

void WindEstimation::updateCalibration()
{
    if (calibrating)
    {
        auto gps = getModule<Sensors>()->getUBXGPSLastSample();

        if (gps.fix != 0)
        {
            if (nSampleCalibration < WES::CALIBRATION_SAMPLE_NUMBER)
            {
                auto gpsVelocity =
                    GeoVelocity{MeterPerSecond{gps.velocityNorth},
                                MeterPerSecond{gps.velocityEast}};

                calibrationMatrix(nSampleCalibration, 0) =
                    MeterPerSecond{gpsVelocity.vn}.value();
                calibrationMatrix(nSampleCalibration, 1) =
                    MeterPerSecond{gpsVelocity.ve}.value();
                calibrationV2(nSampleCalibration) = gpsVelocity.normSquared();

                velocity.vn += gpsVelocity.vn;
                velocity.ve += gpsVelocity.ve;
                speedSquared += gpsVelocity.normSquared();

                nSampleCalibration++;
            }
            else
            {
                velocity.vn /= nSampleCalibration;
                velocity.ve /= nSampleCalibration;
                speedSquared /= nSampleCalibration;

                // Update calibration matrix
                for (int i = 0; i < nSampleCalibration; i++)
                {
                    calibrationMatrix(i, 0) =
                        calibrationMatrix(i, 0) -
                        MeterPerSecond{velocity.vn}.value();
                    calibrationMatrix(i, 1) =
                        calibrationMatrix(i, 1) -
                        MeterPerSecond{velocity.ve}.value();
                    calibrationV2(i) = 0.5f * (calibrationV2(i) - speedSquared);
                }

                Eigen::MatrixXf calibrationMatrixT =
                    calibrationMatrix.transpose();
                auto calibration =
                    (calibrationMatrixT * calibrationMatrix)
                        .ldlt()
                        .solve(calibrationMatrixT * calibrationV2);
                calibratingWind.vn = MeterPerSecond(calibration(0));
                calibratingWind.ve = MeterPerSecond(calibration(1));

                stopCalibration();
                startAlgorithm();
            }
        }
    }
}

void WindEstimation::startAlgorithm()
{
    if (!running)
    {
        running          = true;
        nSampleAlgorithm = nSampleCalibration;

        LOG_INFO(logger, "Wind estimation algorithm started");
    }
    else
    {
        LOG_WARN(logger, "Wind estimation algorithm already started");
    }
}

void WindEstimation::stopAlgorithm()
{
    if (running)
    {
        running = false;

        LOG_INFO(logger, "Wind estimation algorithm stopped");
    }
    else
    {
        LOG_WARN(logger, "Wind estimation algorithm already stopped");
    }
}

void WindEstimation::updateAlgorithm()
{
    if (running)
    {
        auto gps = getModule<Sensors>()->getUBXGPSLastSample();
        if (gps.fix != 0)
        {
            Eigen::Vector2f phi;
            Eigen::Matrix<float, 1, 2> phiT;
            Eigen::Vector2f temp;

            float y;

            auto gpsVelocity = GeoVelocity{MeterPerSecond{gps.velocityNorth},
                                           MeterPerSecond{gps.velocityEast}};

            // update avg
            nSampleAlgorithm++;
            velocity.vn = (velocity.vn * nSampleAlgorithm + gpsVelocity.vn) /
                          (nSampleAlgorithm + 1);
            velocity.ve = (velocity.ve * nSampleAlgorithm + gpsVelocity.ve) /
                          (nSampleAlgorithm + 1);
            speedSquared = (speedSquared * nSampleAlgorithm +
                            (gpsVelocity.normSquared())) /
                           (nSampleAlgorithm + 1);
            phi(0) = MeterPerSecond{gpsVelocity.vn}.value() -
                     MeterPerSecond{velocity.vn}.value();
            phi(1) = MeterPerSecond{gpsVelocity.ve}.value() -
                     MeterPerSecond{velocity.ve}.value();
            y = 0.5f * ((gpsVelocity.normSquared()) - speedSquared);

            phiT = phi.transpose();
            funv =
                (funv - (funv * phi * phiT * funv) / (1 + (phiT * funv * phi)));
            temp = (0.5 * (funv + funv.transpose()) * phi) *
                   (y - phiT * getPrediction().asVector());

            {
                miosix::Lock<FastMutex> l(mutex);
                wind.vn = wind.vn + MeterPerSecond(temp(0));
                wind.ve = wind.ve + MeterPerSecond(temp(1));
            }

            logStatus();
        }
    }
}

GeoVelocity WindEstimation::getPrediction()
{
    miosix::Lock<FastMutex> l(mutex);
    return wind;
}

void WindEstimation::logStatus()
{
    auto timestamp = TimestampTimer::getTimestamp();

    WindEstimationData data{
        .timestamp     = timestamp,
        .velocityNorth = wind.vn,
        .velocityEast  = wind.ve,
        .calibration   = calibrating,
    };

    Logger::getInstance().log(data);
}

}  // namespace Parafoil
