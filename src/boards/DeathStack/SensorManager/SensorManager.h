/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#include <interfaces-impl/hwmapping.h>
#include <math/Stats.h>
#include <scheduler/TaskScheduler.h>
#include <sensors/SensorSampler.h>

#include <vector>

#include "ADA/ADAController.h"
#include "Common.h"
#include "LoggerService/LoggerService.h"
#include "SensorManagerData.h"
#include "configs/SensorManagerConfig.h"
#include "events/FSM.h"

using miosix::PauseKernelLock;
using std::vector;

namespace DeathStackBoard
{

#ifdef USE_MOCK_SENSORS
class MockPressureSensor;
class MockGPS;
#endif

/**
 * The SensorManager class manages all the sensors connected to the Homeone
 * Board.
 *
 * Sensors are grouped by "type" (Simple or DMA) and "sample frequency" and
 * grouped in various SensorSampler objects. These objects are then added to the
 * scheduler that manages the timings for the sampling.
 * After a SensorSampler has finished sampling its sensors, it will call a
 * callback, where these samples can be processed and dispatched.
 */
class SensorManager : public FSM<SensorManager>
{
public:
    SensorManager();
    ~SensorManager();

    bool start() override;

private:
    /**
     * @brief Sensor manager state machine entry state
     *
     */
    void state_idle(const Event& ev);

    /**
     * @brief Sensor manager state machine calibration state
     *
     */
    void state_calibration(const Event& ev);

    /**
     * @brief Sensor manager state machine sampling state
     *
     */
    void state_active(const Event& ev);

    // Logger ref
    LoggerService& logger;

    bool enable_sensor_logging = false;

    SensorManagerStatus status;
};

}  // namespace DeathStackBoard
