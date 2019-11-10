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

#include "DeathStack/System/StackLogger.h"
#include "DeathStack/events/Events.h"
#include "DeathStack/events/Topics.h"
#include "Sensors/Test/TestSensor.h"
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
#include "sensors/MS580301BA07/MS580301BA07.h"

#ifdef USE_MOCK_SENSORS
#include "Sensors/Test/MockGPS.h"
#include "Sensors/Test/MockPressureSensor.h"
#endif

#include "Debug.h"

#include "interfaces-impl/hwmapping.h"

using miosix::FastMutex;
using miosix::Lock;

using namespace miosix;

namespace DeathStackBoard
{

SensorManager::SensorManager(ADAController* ada_controller)
    : FSM(&SensorManager::stateIdle), scheduler(),
      logger(*LoggerService::getInstance()), ada_controller(ada_controller)
{
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_TC);

    memset(&sensor_status, 0, sizeof(sensor_status));

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
    spiMS5803::init();

    // Instantiation
    adc_ad7994        = new AD7994Wrapper(sensors::ad7994::addr, AD7994_V_REF);
    temp_lm75b_analog = new LM75BType(sensors::lm75b_analog::addr);
    temp_lm75b_imu    = new LM75BType(sensors::lm75b_imu::addr);
    pressure_ms5803   = new MS580301BA07Type();

    imu_mpu9250 =
        new MPU9250Type(MPU9250Type::ACC_FS_16G, MPU9250Type::GYRO_FS_2000);

    // imu_adis16405 = new ADIS16405Type(ADIS16405Type::GYRO_FS_300);
    adc_internal = new ADCWrapper();

    piksi = new Piksi("/dev/gps");

#ifdef USE_MOCK_SENSORS
    mock_pressure_sensor = new MockPressureSensor();
    mock_gps             = new MockGPS();
#endif

    // Some sensors dont have init or self tests
    sensor_status.piksi = 1;

    // Initialization
    TRACE("Mpu init\n");
    sensor_status.mpu9250 = imu_mpu9250->init();

    // sensor_status.adis    = imu_adis16405->init();
    TRACE("LM75b IMU init\n");

    sensor_status.lm75b_imu = temp_lm75b_imu->init();

    TRACE("LM75b ANAL init\n");

    sensor_status.lm75b_analog = temp_lm75b_analog->init();

    // // TODO: lsm6ds3h
    TRACE("MS5803 init\n");

    sensor_status.ms5803 = pressure_ms5803->init();
    TRACE("AD7994 init\n");

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

    sampler_20hz_simple.AddSensor(temp_lm75b_imu);
    sampler_20hz_simple.AddSensor(temp_lm75b_analog);

    sampler_50hz_simple.AddSensor(pressure_ms5803);

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
    long long start_time = miosix::getTick() + 10;

    // 250 Hz sensor sampler
    std::function<void()> simple_250hz_callback =
        std::bind(&SensorManager::onSimple250HZCallback, this);
    std::function<void()> simple_250hz_sampler =
        std::bind(&SimpleSensorSampler::UpdateAndCallback,
                  &sampler_250hz_simple, simple_250hz_callback);

    scheduler.add(simple_250hz_sampler, 2,
                  static_cast<uint8_t>(SensorSamplerId::SIMPLE_250HZ),
                  start_time);

    // 100 Hz Calback ( MPU Magnetometer )
    std::function<void()> simple_100hz_callback =
        std::bind(&SensorManager::onSimple100HZCallback, this);

    scheduler.add(simple_100hz_callback, 10,
                  static_cast<uint8_t>(SensorSamplerId::MPU_MAGN_100HZ),
                  start_time);

    // 50 Hz sensor sampler
    std::function<void()> simple_50hz_callback =
        std::bind(&SensorManager::onSimple50HZCallback, this);
    std::function<void()> simple_50hz_sampler =
        std::bind(&SimpleSensorSampler::UpdateAndCallback, &sampler_50hz_simple,
                  simple_50hz_callback);

    scheduler.add(simple_50hz_sampler, 20,
                  static_cast<uint8_t>(SensorSamplerId::SIMPLE_50HZ),
                  start_time);

    // 20 Hz sensor sampler
    std::function<void()> simple_20hz_callback =
        std::bind(&SensorManager::onSimple20HZCallback, this);
    std::function<void()> simple_20hz_sampler =
        std::bind(&SimpleSensorSampler::UpdateAndCallback, &sampler_20hz_simple,
                  simple_20hz_callback);

    scheduler.add(simple_20hz_sampler, 50,
                  static_cast<uint8_t>(SensorSamplerId::SIMPLE_20HZ),
                  start_time);

    // Lambda expression to collect data from GPS at 10 Hz
    std::function<void()> gps_callback =
        std::bind(&SensorManager::onGPSCallback, this);

    scheduler.add(gps_callback, 100, static_cast<uint8_t>(SensorSamplerId::GPS),
                  start_time);

    // Lambda expression callback to log scheduler stats, at 1 Hz
    scheduler.add(
        [&]() {
            scheduler_stats = scheduler.getTaskStats();

            for (TaskStatResult stat : scheduler_stats)
                logger.log(stat);

            StackLogger::getInstance()->updateStack(THID_SENSOR_SAMPLER);
        },
        1000, static_cast<uint8_t>(SensorSamplerId::STATS), start_time);

    TRACE("[SM] Scheduler initialization complete\n");
}

void SensorManager::stateIdle(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            enable_sensor_logging = false;

            status.timestamp = miosix::getTick();
            status.state     = SensorManagerState::IDLE;
            logger.log(status);

            TRACE("[SM] Entering stateIdle\n");

            StackLogger::getInstance()->updateStack(THID_SENSOR_MANAGER);

            break;
        case EV_EXIT:
            TRACE("[SM] Exiting stateIdle\n");

            break;

        // Perform the transition in both cases
        case EV_TC_START_SENSOR_LOGGING:
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
            status.timestamp      = miosix::getTick();
            status.state          = SensorManagerState::LOGGING;
            logger.log(status);

            TRACE("[SM] Entering stateLogging\n");

            StackLogger::getInstance()->updateStack(THID_SENSOR_MANAGER);

            break;
        case EV_EXIT:

            TRACE("[SM] Exiting stateLogging\n");

            break;
#ifdef USE_MOCK_SENSORS
        // Signal to the mock pressure sensor that we have liftoff in order
        // to start simulating flight pressures
        case EV_LIFTOFF:
            mock_pressure_sensor->before_liftoff = false;
            mock_gps->before_liftoff             = false;
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
    LM75BData temp_data;
    temp_data.timestamp   = miosix::getTick();
    temp_data.temp_analog = temp_lm75b_analog->getTemp();
    temp_data.temp_imu    = temp_lm75b_imu->getTemp();

#ifdef USE_MOCK_SENSORS
    ad7994_data->nxp_baro_pressure = mock_pressure_sensor->getPressure();
#endif

    if (enable_sensor_logging)
    {
        logger.log(*(adc_internal->getBatterySensorPtr()->getBatteryDataPtr()));
        logger.log(*(adc_internal->getCurrentSensorPtr()->getCurrentDataPtr()));
        logger.log(*(ad7994_data));

        logger.log(temp_data);
    }

    ada_controller->updateBaro(ad7994_data->nxp_baro_pressure);
}

void SensorManager::onSimple50HZCallback()
{
    if (enable_sensor_logging)
    {
        // Since sampling both temps & pressure on the ms5803 takes two calls of
        // onSimpleUpdate(), log only once every 2
        if (pressure_ms5803->getState() ==
            MS580301BA07Type::STATE_SAMPLED_PRESSURE)
        {
            logger.log(pressure_ms5803->getData());
        }
    }
}

void SensorManager::onSimple100HZCallback()
{
    imu_mpu9250->updateMagneto();

    // Don't log here, magnetometer data will be logged in the 250 hz task with
    // the accel & gyro data.
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

    ada_controller->updateAcc(mpu9255_data.accel.getZ());
}

void SensorManager::onGPSCallback()
{
    PiksiData data;
    memset(&data, 0, sizeof(data));

    try
    {
        data.gps_data = piksi->getGpsData();

        long long fix_age = getTick() - data.gps_data.timestamp;
        // We have fix if the GPS sample is not too old and the number of
        // satellites is at least 4
        data.fix =
            fix_age <= MAX_GPS_FIX_AGE && data.gps_data.numSatellites >= 4;

        last_gps_timestamp = data.gps_data.timestamp;
    }
    catch (std::runtime_error rterr)
    {
        data.gps_data.timestamp = miosix::getTick();
        data.fix                = false;
    }

#ifdef USE_MOCK_SENSORS
    mock_gps->updateCoordinates();
    data.gps_data.timestamp = miosix::getTick();
    data.gps_data.latitude  = mock_gps->lat;
    data.gps_data.longitude = mock_gps->lon;
    data.fix                = mock_gps->fix;
#endif

    ada_controller->updateGPS(data.gps_data.latitude, data.gps_data.longitude,
                              data.fix);

    if (enable_sensor_logging)
    {
        logger.log(data);
    }
}

}  // namespace DeathStackBoard
