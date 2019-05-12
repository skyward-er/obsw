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

#include <stdexcept>

#include "SensorManager.h"

#include "DeathStack/Events.h"
#include "DeathStack/Topics.h"
#include "TestSensor.h"
#include "events/EventBroker.h"

#include <math/Stats.h>

#include "Sensors/AD7994Wrapper.h"
#include "Sensors/ADCWrapper.h"
#include "Sensors/PiksiData.h"
#include "drivers/piksi/piksi.h"
#include "sensors/ADIS16405/ADIS16405.h"
#include "sensors/LM75B.h"
#include "sensors/MPU9250/MPU9250.h"
#include "sensors/MPU9250/MPU9250Data.h"
#include "SensorManagerData.h"

#include "Debug.h"

#include "interfaces-impl/hwmapping.h"

using miosix::FastMutex;
using miosix::Lock;

using namespace miosix;

namespace DeathStackBoard
{

SensorManager::SensorManager(ADA* ada)
    : FSM(&SensorManager::stateIdle), scheduler(),
      logger(*LoggerProxy::getInstance()), ada(ada)
{
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_TC);

    memset(&sensor_status, 1, sizeof(sensor_status));

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
    // Instantiation
    adc_ad7994 = new AD7994Wrapper(sensors::ad7994::addr);
    temp_lm75b = new LM75BType(sensors::lm75b_analog::addr);

    imu_mpu9250 =
        new MPU9250Type(0, 0);  // TODO: Update with correct parameters

    imu_adis16405 = new ADIS16405Type(ADIS16405Type::GYRO_FS_300);
    adc_internal  = new ADCWrapper();

    piksi = new Piksi("/dev/gps");

    // Some sensors dont have init or self tests
    sensor_status.piksi = 1;

    // Initialization
    sensor_status.mpu9250 = imu_mpu9250->init();
    sensor_status.adis    = imu_adis16405->init();

    sensor_status.lm75b = temp_lm75b->init();

    // TODO: lsm6ds3h
    // TODO: ms5803

    sensor_status.ad7994 = adc_ad7994->init();

    sensor_status.battery_sensor = adc_internal->getBatterySensorPtr()->init();
    sensor_status.current_sensor = adc_internal->getCurrentSensorPtr()->init();

    // Self tests
    sensor_status.mpu9250 &= imu_mpu9250->selfTest();
    sensor_status.adis &= imu_adis16405->selfTest();
    sensor_status.lm75b &= temp_lm75b->selfTest();

    sensor_status.ad7994 &= adc_ad7994->selfTest();

    sensor_status.battery_sensor &=
        adc_internal->getBatterySensorPtr()->selfTest();
    sensor_status.current_sensor &=
        adc_internal->getCurrentSensorPtr()->selfTest();

    // TODO: lsm6ds3h
    // TODO: ms5803

    status.sensor_status = sensor_status.toNumeric();
}

void SensorManager::initSamplers()
{
    sampler_1hz_simple.AddSensor(adc_internal->getBatterySensorPtr());

    sampler_20hz_simple.AddSensor(adc_ad7994);
    sampler_20hz_simple.AddSensor(adc_internal->getCurrentSensorPtr());
    sampler_20hz_simple.AddSensor(temp_lm75b);

    sampler_250hz_dma.AddSensor(imu_mpu9250);
    sampler_250hz_dma.AddSensor(imu_adis16405);

    // Piksi does not inherit from Sensor, so we sample it in a different way
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

    scheduler.add(simple_1hz_sampler, 1000, static_cast<uint8_t>(SensorSamplerId::SIMPLE_1HZ));

    // Simple 20 Hz Sampler callback and scheduler function
    std::function<void()> simple_20hz_callback =
        std::bind(&SensorManager::onSimple20HZCallback, this);
    std::function<void()> simple_20hz_sampler =
        std::bind(&SimpleSensorSampler::UpdateAndCallback, &sampler_20hz_simple,
                  simple_20hz_callback);

    scheduler.add(simple_20hz_sampler, 250,
                  static_cast<uint8_t>(SensorSamplerId::SIMPLE_20HZ));  // TODO: back to 50 ms

    // DMA 250 Hz Sampler callback and scheduler function
    std::function<void()> dma_250hz_callback =
        std::bind(&SensorManager::onDMA250HZCallback, this);
    std::function<void()> dma_250Hz_sampler =
        std::bind(&DMASensorSampler::UpdateAndCallback, &sampler_250hz_dma,
                  dma_250hz_callback);

    scheduler.add(dma_250Hz_sampler, 1000,
                  static_cast<uint8_t>(SensorSamplerId::DMA_250HZ));  // TODO: Back to 4 ms

    // Lambda expression to collect data from GPS at 10 Hz
    std::function<void()> gps_callback =
        std::bind(&SensorManager::onGPSCallback, this);

    scheduler.add(gps_callback, 100, static_cast<uint8_t>(SensorSamplerId::GPS));

    // Lambda expression callback to log scheduler stats, at 1 Hz
    scheduler.add(
        [&]() {
        scheduler_stats = scheduler.getTaskStats();

        for (TaskStatResult stat : scheduler_stats)
            logger.log(stat);
        },
        1000, static_cast<uint8_t>(SensorSamplerId::STATS));

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
    AD7994WrapperData* ad7994_data = adc_ad7994->getDataPtr();
    LM75BData lm78b_data           = {miosix::getTick(), temp_lm75b->getTemp()};

    if (enable_sensor_logging)
    {
        logger.log(*(adc_internal->getCurrentSensorPtr()->getCurrentDataPtr()));
        logger.log(*(ad7994_data));
        logger.log(lm78b_data);
    }

    // TODO: Choose which barometer to use, add temperature
    ada->updateBaro(ad7994_data->nxp_baro_pressure, lm78b_data.temp);
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

void SensorManager::onGPSCallback()
{
    PiksiData data;

    try
    {
        data.gps_data = piksi->getGpsData();

        // We have fix if this sample is different from the previous one and we
        // have at least four satellites
        data.fix =
            data.gps_data.timestamp != last_gps_timestamp && data.gps_data.numSatellites >= 4;
        last_gps_timestamp = data.gps_data.timestamp;
    }
    catch (std::runtime_error rterr)
    {
    }

    ada->updateGPS(data.gps_data.latitude, data.gps_data.longitude, data.gps_data.height, data.fix);

    if (enable_sensor_logging)
    {
        logger.log(data);
    }
}

}  // namespace DeathStackBoard
