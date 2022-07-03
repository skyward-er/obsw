/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio, Matteo Pignataro
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

#include <Payload/Payload.h>
#include <algorithms/NAS/NAS.h>
#include <algorithms/NAS/NASState.h>
#include <scheduler/TaskScheduler.h>
#include <sensors/Sensor.h>

#include <functional>

/**
 * USAGE:
 * NASController* nas = new NASController(bind()...);
 *
 * nas->setInitialOrientation({});
 * nas->setInitialPosition({});
 * nas->init();
 * nas->start();
 *
 * nas->getLastSample();
 */

namespace Payload
{
/**
 * @brief The class expects some parameters that represent the data type
 *
 * @tparam IMU The IMU data type
 * @tparam GPS The GPS data type
 */
template <typename IMU, typename GPS>
class NASController
{
public:
    /**
     * @brief Construct a new NASController object
     *
     * @param imuGetter The imu data getter function (done to avoid sync
     * problems with sensors class)
     * @param gpsGetter The gps data getter function (done to avoid sync
     * problems with sensors class)
     */
    NASController(std::function<IMU()> imuGetter,
                  std::function<GPS()> gpsGetter,
                  Boardcore::TaskScheduler* scheduler,
                  Boardcore::NASConfig config = getDefaultConfig());

    /**
     * @brief Destroy the NASController object
     */
    ~NASController();

    /**
     * @brief Default config getter
     */
    Boardcore::NASConfig getDefaultConfig();

    /**
     * @brief Method to set the initial orientation
     */
    void setInitialOrientation(Eigen::Vector3f orientation);

    /**
     * @brief Method to set the initial gps position
     */
    void setInitialPosition(Eigen::Vector2f position);

    /**
     * @brief Method to initialize the controller
     */
    void init();

    /**
     * @brief Method to start the nas
     */
    bool start();

    /**
     * @brief Method to get the last sample
     */
    Boardcore::NASState getLastSample();

    /**
     * @brief Method to make the nas go forward with the prediction and
     * correction
     */
    void step();

private:
    // The NAS filter
    Boardcore::NAS* nas;

    // The nas config
    Boardcore::NASConfig config;
    Eigen::Vector3f initialOrientation;
    Eigen::Vector2f initialPosition;

    // Getter functions
    std::function<IMU> imuGetter;
    std::function<GPS> gpsGetter;

    // Task scheduler
    Boardcore::TaskScheduler* scheduler;

    // SD Logger
    Boardcore::Logger* SDlogger = &Boardcore::Logger::getInstance();

    // Debug logger
    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("NASController");
};
}  // namespace Payload