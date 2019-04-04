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

#include "SensorManager.h"

#include "DeathStack/Events.h"
#include "DeathStack/Topics.h"
#include "TestSensor.h"
#include "events/EventBroker.h"

#include <math/Stats.h>

#include "Sensors/AD7994Wrapper.h"
#include "Sensors/ADCWrapper.h"
#include "sensors/ADIS16405/ADIS16405.h"
#include "sensors/MPU9250/MPU9250.h"
#include "sensors/MPU9250/MPU9250Data.h"

#include "Debug.h"

using miosix::FastMutex;
using miosix::Lock;

namespace DeathStackBoard
{

SensorManager::SensorManager(ADA* ada)
    : FSM(&SensorManager::stateIdle), scheduler(),
      logger(*LoggerProxy::getInstance()), ada(ada)
{
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_TC);

    initSensors();
    initSamplers();

    scheduler.start();
}

SensorManager::~SensorManager()
{
    sEventBroker->unsubscribe(this);
    scheduler.stop();
}

void SensorManager::initSensors()
{
    adc_ad7994 = new AD7994Wrapper(AD7994_I2C_ADDRESS);

    imu_mpu9250 =
        new MPU9250Type(0, 0);  // TODO: Update with correct parameters
    if (!imu_mpu9250->init())
    {
        status.problematic_sensors |= SENSOR_MPU9255;
    }

    imu_adis16405 = new ADIS16405Type();
    imu_adis16405->init();

    adc_internal = new ADCWrapper();

    // TODO: Self tests
}

void SensorManager::initSamplers()
{
    sampler_1hz_simple.AddSensor(adc_internal->getBatterySensorPtr());

    sampler_20hz_simple.AddSensor(adc_ad7994);
    sampler_20hz_simple.AddSensor(adc_internal->getCurrentSensorPtr());

    sampler_250hz_dma.AddSensor(imu_mpu9250);
    sampler_250hz_dma.AddSensor(imu_adis16405);
}

void SensorManager::stateIdle(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            enable_sensor_logging = false;

            TRACE("SM: Entering stateIdle\n");
            status.state = SensorManagerState::IDLE;
            logger.log(status);
            break;
        case EV_EXIT:
            break;

        // Perform the transition in both cases
        case EV_TC_START_LOGGING:
        case EV_ARMED:
            transition(&SensorManager::stateLogging);
            break;

        default:
            break;
    }
}

void SensorManager::stateLogging(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            enable_sensor_logging = true;

            TRACE("SM: Entering stateLogging\n");
            status.state = SensorManagerState::LOGGING;
            logger.log(status);
            break;
        case EV_EXIT:
            break;

        // Go back to idle in both cases
        case EV_TC_STOP_LOGGING:
        case EV_LANDED:
            transition(&SensorManager::stateIdle);
            break;

        default:
            break;
    }
}

void SensorManager::startSampling()
{
    /*
     * std::bind syntax:
     * std::bind(&MyClass::someFunction, &myclass_instance, [someFunction args])
     */

    // Simple 1 Hz Sampler callback and scheduler function
    std::function<void()> simple_1hz_callback =
        std::bind(&SensorManager::onSimple1HZCallback, this);
    std::function<void()> simple_1hz_sampler =
        std::bind(&SimpleSensorSampler::UpdateAndCallback, &sampler_1hz_simple,
                  simple_1hz_callback);

    scheduler.add(simple_1hz_sampler, 1000, ID_SIMPLE_1HZ);

    // Simple 20 Hz Sampler callback and scheduler function
    std::function<void()> simple_20hz_callback =
        std::bind(&SensorManager::onSimple20HZCallback, this);
    std::function<void()> simple_20hz_sampler =
        std::bind(&SimpleSensorSampler::UpdateAndCallback, &sampler_20hz_simple,
                  simple_20hz_callback);

    scheduler.add(simple_20hz_sampler, 250,
                  ID_SIMPLE_20HZ);  // TODO: back to 50 ms

    // DMA 250 Hz Sampler callback and scheduler function
    std::function<void()> dma_250hz_callback =
        std::bind(&SensorManager::onDMA250HZCallback, this);
    std::function<void()> dma_250Hz_sampler =
        std::bind(&DMASensorSampler::UpdateAndCallback, &sampler_250hz_dma,
                  dma_250hz_callback);

    scheduler.add(dma_250Hz_sampler, 1000,
                  ID_DMA_250HZ);  // TODO: Back to 4 ms

    // Lambda expression callback to log scheduler stats, at 1 Hz
    scheduler.add(
        [&]() {
            scheduler_stats = scheduler.getTaskStats();

            for (TaskStatResult stat : scheduler_stats)
                logger.log(stat);
        },
        1000, ID_STATS);

    TRACE("Scheduler initialization complete\n");
}

void SensorManager::onSimple1HZCallback()
{
    // Log the battery voltage level we just finished sampling.
    if (enable_sensor_logging)
    {
        logger.log(*(adc_internal->getBatterySensorPtr()->getBatteryDataPtr()));
    }
}

void SensorManager::onSimple20HZCallback()
{
    AD7994WrapperData ad7994_data = adc_ad7994->getData();

    if (enable_sensor_logging)
    {
        logger.log(*(adc_internal->getCurrentSensorPtr()->getCurrentDataPtr()));
        logger.log(ad7994_data);
    }

    // TODO: Calculate & log barometer stats

    // TODO: Choose which barometer to use
    ada->update(adc_ad7994->getData().baro_1_volt);
}

void SensorManager::onDMA250HZCallback()
{

    MPU9250Data mpu9255_data{
        *(imu_mpu9250->accelDataPtr()), *(imu_mpu9250->gyroDataPtr()),
        *(imu_mpu9250->compassDataPtr()), *(imu_mpu9250->tempDataPtr())};

    if (enable_sensor_logging)
    {
        logger.log(mpu9255_data);

        logger.log(*(imu_adis16405->gyroDataPtr()));
        logger.log(*(imu_adis16405->accelDataPtr()));
        logger.log(*(imu_adis16405->tempDataPtr()));
    }
}

}  // namespace DeathStackBoard
