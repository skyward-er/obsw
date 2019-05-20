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
#include <iostream>

#include "SensorManagerData.h"
#include "Sensors/AD7994Wrapper.h"
#include "Sensors/ADCWrapper.h"
#include "Sensors/PiksiData.h"
#include "drivers/piksi/piksi.h"
#include "sensors/ADIS16405/ADIS16405.h"
#include "sensors/LM75B.h"
#include "sensors/MPU9250/MPU9250.h"
#include "sensors/MPU9250/MPU9250Data.h"

#ifdef USE_MOCK_SENSORS
#include "Sensors/Test/MockPressureSensor.h"
#endif

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

    initScheduler();
}

SensorManager::~SensorManager()
{
    sEventBroker->unsubscribe(this);
    scheduler.stop();
}

bool SensorManager::start()
{
    // Start the parent FSM
    bool ok = FSM<SensorManager>::start();

    // Start the scheduler
    ok = ok && scheduler.start();

    return ok;
}

void SensorManager::initSensors()
{
    i2c1::init();
    spiMPU9250::init();

    // Instantiation
    adc_ad7994        = new AD7994Wrapper(sensors::ad7994::addr);
    temp_lm75b_analog = new LM75BType(sensors::lm75b_analog::addr);
    temp_lm75b_imu    = new LM75BType(sensors::lm75b_imu::addr);

    imu_mpu9250 =
        new MPU9250Type(MPU9250Type::ACC_FS_16G, MPU9250Type::GYRO_FS_2000);

    // imu_adis16405 = new ADIS16405Type(ADIS16405Type::GYRO_FS_300);
    adc_internal = new ADCWrapper();

    piksi = new Piksi("/dev/gps");

#ifdef USE_MOCK_SENSORS
    mock_pressure_sensor = new MockPressureSensor();
#endif

    // Some sensors dont have init or self tests
    sensor_status.piksi = 1;

    // Initialization
    sensor_status.mpu9250 = imu_mpu9250->init();

    // sensor_status.adis    = imu_adis16405->init();

    sensor_status.lm75b_imu    = temp_lm75b_imu->init();
    sensor_status.lm75b_analog = temp_lm75b_analog->init();

    // // TODO: lsm6ds3h
    // // TODO: ms5803

    sensor_status.ad7994 = adc_ad7994->init();

    sensor_status.battery_sensor = adc_internal->getBatterySensorPtr()->init();
    sensor_status.current_sensor = adc_internal->getCurrentSensorPtr()->init();

    status.sensor_status = sensor_status.toNumeric();

    TRACE("[SM] Sensor init done\n");
}

void SensorManager::initSamplers()
{
    sampler_20hz_simple.AddSensor(adc_internal->getBatterySensorPtr());
    sampler_20hz_simple.AddSensor(adc_ad7994);
    sampler_20hz_simple.AddSensor(adc_internal->getCurrentSensorPtr());
#ifdef USE_MOCK_SENSORS
    sampler_20hz_simple.AddSensor(mock_pressure_sensor);
#endif

    sampler_20hz_simple.AddSensor(temp_lm75b_imu);
    sampler_20hz_simple.AddSensor(temp_lm75b_analog);
    sampler_250hz_simple.AddSensor(imu_mpu9250);

    // Piksi does not inherit from Sensor, so we sample it in a different way

    TRACE("[SM] Sampler init done\n");
}

void SensorManager::initScheduler()
{
    /*
     * std::bind syntax:
     * std::bind(&MyClass::someFunction, &myclass_instance, [someFunction args])
     */

    std::function<void()> simple_20hz_callback =
        std::bind(&SensorManager::onSimple20HZCallback, this);
    std::function<void()> simple_20hz_sampler =
        std::bind(&SimpleSensorSampler::UpdateAndCallback, &sampler_20hz_simple,
                  simple_20hz_callback);

    scheduler.add(simple_20hz_sampler, 50,
                  static_cast<uint8_t>(SensorSamplerId::SIMPLE_20HZ));

    std::function<void()> simple_250hz_callback =
        std::bind(&SensorManager::onSimple250HZCallback, this);
    std::function<void()> simple_250hz_sampler =
        std::bind(&SimpleSensorSampler::UpdateAndCallback,
                  &sampler_250hz_simple, simple_250hz_callback);

    scheduler.add(simple_250hz_sampler, 4,
                  static_cast<uint8_t>(SensorSamplerId::SIMPLE_250HZ));

    // Lambda expression to collect data from GPS at 10 Hz
    std::function<void()> gps_callback =
        std::bind(&SensorManager::onGPSCallback, this);

    scheduler.add(gps_callback, 100,
                  static_cast<uint8_t>(SensorSamplerId::GPS));

    // Lambda expression callback to log scheduler stats, at 1 Hz
    scheduler.add(
        [&]() {
            scheduler_stats = scheduler.getTaskStats();

            for (TaskStatResult stat : scheduler_stats)
                logger.log(stat);
        },
        1000, static_cast<uint8_t>(SensorSamplerId::STATS));

    TRACE("[SM] Scheduler initialization complete\n");
}

void SensorManager::stateIdle(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            enable_sensor_logging = false;

            status.state = SensorManagerState::IDLE;
            logger.log(status);

            TRACE("[SM] Entering stateIdle\n");
            break;
        case EV_EXIT:
            TRACE("[SM] Exiting stateIdle\n");

            break;

        // Perform the transition in both cases
        case EV_TC_START_SENSOR_LOGGING:
        case EV_LIFTOFF:

#ifdef USE_MOCK_SENSORS
            // Signal to the mock pressure sensor that we have liftoff in order
            // to start simulating flight pressures
            if (ev.sig == EV_LIFTOFF)
            {
                mock_pressure_sensor->before_liftoff = false;
            }
            break;
#endif
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

            status.state = SensorManagerState::LOGGING;
            logger.log(status);

            TRACE("[SM] Entering stateLogging\n");
            break;
        case EV_EXIT:

            TRACE("[SM] Exiting stateLogging\n");

            break;
#ifdef USE_MOCK_SENSORS
        // Signal to the mock pressure sensor that we have liftoff in order
        // to start simulating flight pressures
        case EV_LIFTOFF:
            mock_pressure_sensor->before_liftoff = false;
            break;
#endif
        // Go back to idle in both cases
        case EV_TC_STOP_SENSOR_LOGGING:
        case EV_LANDED:
            transition(&SensorManager::stateIdle);
            break;

        default:
            break;
    }
}

void SensorManager::onSimple20HZCallback()
{
    AD7994WrapperData* ad7994_data = adc_ad7994->getDataPtr();
    LM75BData lm78b_analog_data    = {TempSensorId::LM75B_ANALOG,
                                   miosix::getTick(),
                                   temp_lm75b_analog->getTemp()};
    LM75BData lm78b_imu_data = {TempSensorId::LM75B_IMU, miosix::getTick(),
                                temp_lm75b_imu->getTemp()};

#ifdef USE_MOCK_SENSORS
    ad7994_data->nxp_baro_pressure = *(mock_pressure_sensor->pressureDataPtr());
#endif

    if (enable_sensor_logging)
    {
        logger.log(*(adc_internal->getBatterySensorPtr()->getBatteryDataPtr()));
        logger.log(*(adc_internal->getCurrentSensorPtr()->getCurrentDataPtr()));
        logger.log(*(ad7994_data));
        logger.log(lm78b_imu_data);
        logger.log(lm78b_analog_data);
    }

    // TODO: Choose which barometer to use, add temperature
    // ada->updateBaro(ad7994_data->nxp_baro_pressure, lm78b_imu_data.temp);
}

void SensorManager::onSimple250HZCallback()
{
    MPU9250Data mpu9255_data{miosix::getTick(), *(imu_mpu9250->accelDataPtr()),
                             *(imu_mpu9250->gyroDataPtr()),
                             *(imu_mpu9250->compassDataPtr()),
                             *(imu_mpu9250->tempDataPtr())};

    if (enable_sensor_logging)
    {
        logger.log(mpu9255_data);
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
        data.fix = data.gps_data.timestamp != last_gps_timestamp &&
                   data.gps_data.numSatellites >= 4;

        last_gps_timestamp = data.gps_data.timestamp;
    }
    catch (std::runtime_error rterr)
    {
        data.fix = false;
    }

    // ada->updateGPS(data.gps_data.latitude, data.gps_data.longitude,
    //                data.gps_data.height, data.fix);

    if (enable_sensor_logging)
    {
        logger.log(data);
    }
}

}  // namespace DeathStackBoard
