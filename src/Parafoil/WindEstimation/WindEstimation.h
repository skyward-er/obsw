/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Federico Mandelli, Davide Basso
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

#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Configs/WESConfig.h>
#include <Parafoil/Sensors/Sensors.h>
#include <diagnostic/PrintLogger.h>
#include <logger/Logger.h>

#include <Eigen/Core>
#include <atomic>
#include <utils/ModuleManager/ModuleManager.hpp>

#include "WindEstimationData.h"

namespace Parafoil
{
class BoardScheduler;
class Sensors;

/**
 * @brief This class implements the Wind Estimation Scheme(WES) prediction
 * algorithm, the first part is the initial setup, and then the continuos
 * algorithms runs;
 */
class WindEstimation
    : public Boardcore::InjectableWithDeps<BoardScheduler, Sensors>
{
public:
    WindEstimation();

    ~WindEstimation();

    /**
     * @brief Starts the wind estimation module, subscribing the calibration and
     * algorithm update tasks to the scheduler.
     * @return true if the module started correctly, false otherwise.
     */
    bool start();

    bool isStarted();

    /**
     * @brief Starts the calibration of the wind estimation.
     */
    void startCalibration();

    /**
     * @brief Stops the calibration of the wind estimation. Overrides the
     * current wind estimation with the calibrated one.
     */
    void stopCalibration();

    /**
     * @brief Starts the algorithm of the wind estimation.
     */
    void startAlgorithm();

    /**
     * @brief Stops the algorithm of the wind estimation.
     */
    void stopAlgorithm();

    /**
     * @brief Returns the current wind estimation prediction.
     */
    GeoVelocity getPrediction();

private:
    /**
     * @brief Updates the calibration matrix with the new calibration values.
     * Automatically stops the calibration when the calibration matrix is full
     * and starts the algorithm.
     */
    void updateCalibration();

    /**
     * @brief Updates the wind matrix with the updated wind prediction values.
     */
    void updateAlgorithm();

    /**
     * @brief Logs the current prediction.
     */
    void logStatus();

    GeoVelocity calibratingWind;  ///< Wind currently being calibrated
    GeoVelocity wind;             ///< Wind after calibration

    // Calibration & Algorithm variables
    uint8_t nSampleCalibration = 0;
    uint8_t nSampleAlgorithm   = 0;

    Eigen::Matrix<float, Config::WES::CALIBRATION_SAMPLE_NUMBER, 2>
        calibrationMatrix;
    Eigen::Vector<float, Config::WES::CALIBRATION_SAMPLE_NUMBER> calibrationV2;
    Eigen::Matrix2f funv;

    GeoVelocity velocity;   ///< Wind velocity (North, East)
    float speedSquared{0};  ///< Wind speed squared

    miosix::FastMutex mutex;

    // Status variables
    std::atomic<bool> started{false};
    std::atomic<bool> running{false};  ///< Whether the algorithm is running
    std::atomic<bool> calibrating{
        false};  ///< Whether the algorithm is being calibrated

    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("WindEstimation");
};
}  // namespace Parafoil
