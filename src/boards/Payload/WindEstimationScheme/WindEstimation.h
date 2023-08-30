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

#pragma once

#include <Payload/BoardScheduler.h>
#include <Payload/Configs/WESConfig.h>
#include <diagnostic/PrintLogger.h>
#include <logger/Logger.h>

#include <Eigen/Core>
#include <atomic>
#include <utils/ModuleManager/ModuleManager.hpp>

#include "WindEstimationData.h"
namespace Payload
{

/**
 * @brief This class implements the wind prediction algorithm, the first part is
 * the initial setup, and then the continuos algoritms runs;
 */
class WindEstimation : public Boardcore::Module
{
public:
    /**
     * @brief Construct a new Wing Controller object
     */
    explicit WindEstimation(Boardcore::TaskScheduler* sched);

    /**
     * @brief Destroy the Wing Controller object.
     */
    ~WindEstimation();

    bool start();

    void startWindEstimationSchemeCalibration();

    void stopWindEstimationSchemeCalibration();

    void startWindEstimationScheme();

    void stopWindEstimationScheme();

    bool getStatus();

    Eigen::Vector2f getWindEstimationScheme();

private:
    /**
     * @brief Creates the windCalibration matrix with the starting prediction
     * value
     */
    void windEstimationSchemeCalibration();

    /**
     * @brief Updates the wind matrix with the updated wind prediction values
     */
    void windEstimationScheme();

    /**
     * @brief Logs the prediction
     */
    void logStatus();

    /**
     * @brief Parameters needed for calibration
     */
    Eigen::Vector2f windCalibration{0.0f, 0.0f};
    uint8_t nSampleCal = 0;
    Eigen::Matrix<float, WESConfig::WES_CALIBRATION_SAMPLE_NUMBER, 2>
        calibrationMatrix;
    Eigen::Vector<float, WESConfig::WES_CALIBRATION_SAMPLE_NUMBER>
        calibrationV2;
    float vx = 0;
    float vy = 0;
    float v2 = 0;

    /**
     * @brief Parameters needed for recursive
     */
    Eigen::Vector2f wind{0.0f, 0.0f};
    uint8_t nSample = 0;
    Eigen::Matrix2f funv;

    /**
     * @brief Mutex
     */
    miosix::FastMutex mutex;

    /**
     * @brief PrintLogger
     */
    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("PayloadTest");

    /**
     * @brief Logging struct
     */
    WindLogging windLogger{0, 0, 0};

    /**
     * @brief Internal running state
     */
    std::atomic<bool> running;
    std::atomic<bool> calRunning;

    Boardcore::TaskScheduler* scheduler = nullptr;
};
}  // namespace Payload
