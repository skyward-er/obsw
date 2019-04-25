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

#include <vector>
#include "Common.h"

#include <math/Stats.h>
#include <sensors/SensorSampling.h>
#include <scheduler/TaskScheduler.h>

#include "DeathStack/LogProxy/LogProxy.h"
#include "DeathStack/configs/SensorManagerConfig.h"
#include "events/FSM.h"

#include "SensorManagerData.h"
#include "DeathStack/ADA/ADA.h"

using miosix::PauseKernelLock;
using std::vector;

// Forward declarations
template <typename BusSPI>
class MPU9250;

template <typename BusSPI>
class MAX21105;

template <typename BusSPI>
class ADIS16405;

class Piksi;

namespace DeathStackBoard
{
class ADCWrapper;
class AD7994Wrapper;

// Type definitions
typedef MPU9250<spiMPU9250> MPU9250Type;
typedef ADIS16405<spiADIS16405> ADIS16405Type;

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
    enum SensorSamplerId : uint8_t
    {
        ID_SIMPLE_1HZ,
        ID_SIMPLE_20HZ,
        ID_DMA_250HZ,
        ID_GPS,
        ID_STATS
    };

    SensorManager(ADA* ada);
    ~SensorManager();

    vector<TaskStatResult> getSchedulerStats()
    {
        return scheduler_stats;
    }

    SensorManagerStatus getStatus()
    {
        return status;
    }

private:
    /**
     * Initialize all the sensors.
     */
    void initSensors();

    /**
     * Initialize the samplers.
     */
    void initSamplers();

    /**
     * @brief Sensor manager state machine entry state
     *
     */
    void stateIdle(const Event& ev);

    /**
     * @brief Sensor manager state machine sampling state
     *
     */
    void stateLogging(const Event& ev);

    /**
     * Adds all the SensorSamplers to the scheduler and begins sampling.
     */
    void startSampling();

    /*
     * Callbacks. These functions are called each time the corresponding
     * SensorSampler has acquired new samples.
     * These functions are called on the scheduler (sampler) thread, so avoid
     * performing non-critical and intensive tasks.
     */

    /**
     * @brief Simple, 1 Hz SensorSampler Callback.
     * Called each time all the sensors in the 1hz sampler have been sampled
     */
    void onSimple1HZCallback();

    /**
     * @brief Simple, 20 Hz SensorSampler Callback.
     * Called each time all the sensors in the 20hz sampler have been sampled
     */
    void onSimple20HZCallback();

    /**
     * @brief DMA, 250 Hz SensorSampler Callback.
     * Called each time all the sensors in the 250hz sampler have been sampled
     */
    void onDMA250HZCallback();

    void onGPSCallback();

    TaskScheduler scheduler;
    // Logger ref
    LoggerProxy& logger;

    bool enable_sensor_logging = false;

    // Sensor samplers
    SimpleSensorSampler sampler_1hz_simple;
    SimpleSensorSampler sampler_20hz_simple;
    DMASensorSampler sampler_250hz_dma;

    // Sensors
    AD7994Wrapper* adc_ad7994;
    MPU9250Type* imu_mpu9250;
    ADIS16405Type* imu_adis16405;
    ADCWrapper* adc_internal;
    Piksi* piksi;

    //ADA
    ADA* ada;

    // Stats & status
    vector<TaskStatResult> scheduler_stats;
    SensorManagerStatus status;
};

}  // namespace DeathStackBoard
