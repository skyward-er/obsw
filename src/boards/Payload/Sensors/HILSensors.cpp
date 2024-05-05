/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro, Emilio Corigliano
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

#include "HILSensors.h"

#include <Payload/Buses.h>
#include <Payload/Configs/HILSimulationConfig.h>
#include <Payload/Configs/SensorsConfig.h>
#include <common/ReferenceConfig.h>
#include <interfaces-impl/hwmapping.h>

using namespace Boardcore;
using namespace std;
using namespace Payload::SensorsConfig;
using namespace HILConfig;

namespace Payload
{
LPS22DFData HILSensors::getLPS22DFLastSample()
{
    miosix::PauseKernelLock lock;
    return (lps22df != nullptr
                ? LPS22DFData{lps22df->getLastSample().pressureTimestamp,
                              lps22df->getLastSample().pressure,
                              temperature->getLastSample().temperature}
                : LPS22DFData{});
}
LPS28DFWData HILSensors::getLPS28DFW_1LastSample()
{
    miosix::PauseKernelLock lock;
    return lps28dfw_1 != nullptr
               ? LPS28DFWData{lps28dfw_1->getLastSample().pressureTimestamp,
                              lps28dfw_1->getLastSample().pressure,
                              temperature->getLastSample().temperatureTimestamp,
                              temperature->getLastSample().temperature}
               : LPS28DFWData{};
}
LPS28DFWData HILSensors::getLPS28DFW_2LastSample()
{
    miosix::PauseKernelLock lock;
    return lps28dfw_2 != nullptr
               ? LPS28DFWData{lps28dfw_2->getLastSample().pressureTimestamp,
                              lps28dfw_2->getLastSample().pressure,
                              temperature->getLastSample().temperatureTimestamp,
                              temperature->getLastSample().temperature}
               : LPS28DFWData{};
}
H3LIS331DLData HILSensors::getH3LIS331DLLastSample()
{
    miosix::PauseKernelLock lock;
    return h3lis331dl != nullptr
               ? H3LIS331DLData{static_cast<Boardcore::AccelerometerData>(
                     h3lis331dl->getLastSample())}
               : H3LIS331DLData{};
}
LIS2MDLData HILSensors::getLIS2MDLLastSample()
{
    miosix::PauseKernelLock lock;
    return lis2mdl != nullptr
               ? LIS2MDLData{static_cast<Boardcore::MagnetometerData>(
                                 lis2mdl->getLastSample()),
                             static_cast<Boardcore::TemperatureData>(
                                 temperature->getLastSample())}
               : LIS2MDLData{};
}
UBXGPSData HILSensors::getGPSLastSample()
{
    if (ubxgps == nullptr)
    {
        return UBXGPSData{};
    }
    UBXGPSData lastSample;

    miosix::PauseKernelLock lock;
    GPSData gps = static_cast<Boardcore::GPSData>(ubxgps->getLastSample());
    lastSample.ubxTime       = UBXDateTime{};
    lastSample.gpsTimestamp  = gps.gpsTimestamp;
    lastSample.latitude      = gps.latitude;
    lastSample.longitude     = gps.longitude;
    lastSample.height        = gps.height;
    lastSample.velocityNorth = gps.velocityNorth;
    lastSample.velocityEast  = gps.velocityEast;
    lastSample.velocityDown  = gps.velocityDown;
    lastSample.speed         = gps.speed;
    lastSample.track         = gps.track;
    lastSample.positionDOP   = gps.positionDOP;
    lastSample.satellites    = gps.satellites;
    lastSample.fix           = gps.fix;
    return lastSample;
}
LSM6DSRXData HILSensors::getLSM6DSRXLastSample()
{
    if (lsm6dsrx_accel == nullptr || lsm6dsrx_gyro == nullptr)
    {
        return LSM6DSRXData{};
    }

    LSM6DSRXData lastSample;

    HILAccelerometerData accelData;
    HILGyroscopeData gyroData;
    {
        miosix::PauseKernelLock lock;
        accelData = lsm6dsrx_accel->getLastSample();
        gyroData  = lsm6dsrx_gyro->getLastSample();
    }

    lastSample.accelerationTimestamp = accelData.accelerationTimestamp;
    lastSample.accelerationX         = accelData.accelerationX;
    lastSample.accelerationY         = accelData.accelerationY;
    lastSample.accelerationZ         = accelData.accelerationZ;

    lastSample.angularSpeedTimestamp = gyroData.angularSpeedTimestamp;
    lastSample.angularSpeedX         = gyroData.angularSpeedX;
    lastSample.angularSpeedY         = gyroData.angularSpeedY;
    lastSample.angularSpeedZ         = gyroData.angularSpeedZ;

    return lastSample;
}

Boardcore::HSCMRNN015PAData HILSensors::getStaticPressureLastSample()
{
    Boardcore::HSCMRNN015PAData data;
    miosix::PauseKernelLock lock;

    auto baro              = getLPS22DFLastSample();
    data.pressureTimestamp = baro.pressureTimestamp;
    data.pressure          = baro.pressure;

    return data;
}

Boardcore::SSCMRNN030PAData HILSensors::getDynamicPressureLastSample()
{
    Boardcore::SSCMRNN030PAData data;
    miosix::PauseKernelLock lock;

    auto baro              = dynamicPressure->getLastSample();
    data.pressureTimestamp = baro.pressureTimestamp;
    data.pressure          = baro.pressure;

    return data;
}

HILSensors::HILSensors(TaskScheduler* sched) : Payload::Sensors(sched) {}

bool HILSensors::start()
{
    // Init all the sensors
    temperatureInit();
    LOG_INFO(logger, "temperatureInit\n");
    lps22dfInit();
    LOG_INFO(logger, "lps22dfInit\n");
    lps28dfw_1Init();
    LOG_INFO(logger, "lps28dfw_1Init\n");
    lps28dfw_2Init();
    LOG_INFO(logger, "lps28dfw_2Init\n");
    h3lis331dlInit();
    LOG_INFO(logger, "h3lis331dlInit\n");
    lis2mdlInit();
    LOG_INFO(logger, "lis2mdlInit\n");
    ubxgpsInit();
    LOG_INFO(logger, "ubxgpsInit\n");
    lsm6dsrxInit();
    LOG_INFO(logger, "lsm6dsrxInit\n");
    // ads131m08Init();
    // LOG_INFO(logger, "ads131m08Init\n");
    staticPressureInit();
    LOG_INFO(logger, "staticPressureInit\n");
    dynamicPressureInit();
    LOG_INFO(logger, "dynamicPressureInit\n");
    pitotInit();
    LOG_INFO(logger, "pitotInit\n");
    imuInit();
    LOG_INFO(logger, "imuInit\n");

    // Add the magnetometer calibration to the scheduler
    size_t result = scheduler->addTask(
        [&]()
        {
            // Gather the last sample data
            MagnetometerData lastSample = getLIS2MDLLastSample();

            // Feed the data to the calibrator inside a protected area.
            // Contention is not high and the use of a mutex is suitable to
            // avoid pausing the kernel for this calibration operation
            {
                miosix::Lock<FastMutex> l(calibrationMutex);
                magCalibrator.feed(lastSample);
            }
        },
        MAG_CALIBRATION_PERIOD);

    // Create sensor manager with populated map and configured scheduler
    manager = new SensorManager(sensorMap, scheduler);
    return manager->start() && result != 0;
}

void HILSensors::temperatureInit()
{
    temperature =
        new PayloadHILTemperature(&Boardcore::ModuleManager::getInstance()
                                       .get<PayloadHIL>()
                                       ->hilTransceiver->getSensorData()
                                       ->temperature);

    // Emplace the sensor inside the map
    SensorInfo info("TEMP_PayloadHIL", 1000 / TEMP_FREQ);

    sensorMap.emplace(make_pair(temperature, info));
}

void HILSensors::lps22dfInit()
{
    // Create sensor instance with configured parameters
    lps22df =
        new PayloadHILDigitalBarometer(&Boardcore::ModuleManager::getInstance()
                                            .get<PayloadHIL>()
                                            ->hilTransceiver->getSensorData()
                                            ->barometer1);

    // Emplace the sensor inside the map
    SensorInfo info("LPS22DF_PayloadHIL", LPS22DF_PERIOD,
                    bind(&HILSensors::lps22dfCallback, this));
    sensorMap.emplace(make_pair(lps22df, info));
}

void HILSensors::lps28dfw_1Init()
{
    // Create sensor instance with configured parameters
    lps28dfw_1 =
        new PayloadHILDigitalBarometer(&Boardcore::ModuleManager::getInstance()
                                            .get<PayloadHIL>()
                                            ->hilTransceiver->getSensorData()
                                            ->barometer2);

    // Emplace the sensor inside the map
    SensorInfo info("LPS28DFW_1_PayloadHIL", LPS28DFW_PERIOD,
                    bind(&HILSensors::lps28dfw_1Callback, this));
    sensorMap.emplace(make_pair(lps28dfw_1, info));
}

void HILSensors::lps28dfw_2Init()
{
    // Create sensor instance with configured parameters
    lps28dfw_2 =
        new PayloadHILDigitalBarometer(&Boardcore::ModuleManager::getInstance()
                                            .get<PayloadHIL>()
                                            ->hilTransceiver->getSensorData()
                                            ->barometer3);

    // Emplace the sensor inside the map
    SensorInfo info("LPS28DFW_2_PayloadHIL", LPS28DFW_PERIOD,
                    bind(&HILSensors::lps28dfw_2Callback, this));
    sensorMap.emplace(make_pair(lps28dfw_2, info));
}

void HILSensors::h3lis331dlInit()
{
    // Create sensor instance with configured parameters
    h3lis331dl =
        new PayloadHILAccelerometer(&Boardcore::ModuleManager::getInstance()
                                         .get<PayloadHIL>()
                                         ->hilTransceiver->getSensorData()
                                         ->accelerometer);

    // Emplace the sensor inside the map
    SensorInfo info("H3LIS331DL_PayloadHIL", H3LIS331DL_PERIOD,
                    bind(&HILSensors::h3lis331dlCallback, this));
    sensorMap.emplace(make_pair(h3lis331dl, info));
}

void HILSensors::lis2mdlInit()
{
    // Create sensor instance with configured parameters
    lis2mdl =
        new PayloadHILMagnetometer(&Boardcore::ModuleManager::getInstance()
                                        .get<PayloadHIL>()
                                        ->hilTransceiver->getSensorData()
                                        ->magnetometer);

    // Emplace the sensor inside the map
    SensorInfo info("LIS2MDL_PayloadHIL", LIS2MDL_PERIOD,
                    bind(&HILSensors::lis2mdlCallback, this));
    sensorMap.emplace(make_pair(lis2mdl, info));
}

void HILSensors::ubxgpsInit()
{
    // Create sensor instance with configured parameters
    ubxgps = new PayloadHILGps(&Boardcore::ModuleManager::getInstance()
                                    .get<PayloadHIL>()
                                    ->hilTransceiver->getSensorData()
                                    ->gps);

    // Emplace the sensor inside the map
    SensorInfo info("UBXGPS_PayloadHIL", UBXGPS_PERIOD,
                    bind(&HILSensors::ubxgpsCallback, this));
    sensorMap.emplace(make_pair(ubxgps, info));
}

void HILSensors::lsm6dsrxInit()
{
    // Create sensor instance with configured parameters
    lsm6dsrx_accel =
        new PayloadHILAccelerometer(&Boardcore::ModuleManager::getInstance()
                                         .get<PayloadHIL>()
                                         ->hilTransceiver->getSensorData()
                                         ->accelerometer);
    lsm6dsrx_gyro =
        new PayloadHILGyroscope(&Boardcore::ModuleManager::getInstance()
                                     .get<PayloadHIL>()
                                     ->hilTransceiver->getSensorData()
                                     ->gyro);

    // Emplace the sensor inside the map
    SensorInfo info("LSM6DSRX", LSM6DSRX_PERIOD,
                    bind(&HILSensors::lsm6dsrxCallback, this));
    sensorMap.emplace(make_pair(lsm6dsrx_accel, info));
    sensorMap.emplace(make_pair(lsm6dsrx_gyro, info));
}

void HILSensors::staticPressureInit()
{
    // Create sensor instance with configured parameters
    staticPressure =
        new PayloadHILAnalogBarometer(&Boardcore::ModuleManager::getInstance()
                                           .get<PayloadHIL>()
                                           ->hilTransceiver->getSensorData()
                                           ->staticPitot);

    // Emplace the sensor inside the map
    SensorInfo info("StaticPitot", 1000 / ANALOG_BARO_FREQ,
                    bind(&HILSensors::staticPressureCallback, this));
    sensorMap.emplace(make_pair(staticPressure, info));
}

void HILSensors::dynamicPressureInit()
{
    // Create sensor instance with configured parameters
    dynamicPressure =
        new PayloadHILAnalogBarometer(&Boardcore::ModuleManager::getInstance()
                                           .get<PayloadHIL>()
                                           ->hilTransceiver->getSensorData()
                                           ->dynamicPitot);

    // Emplace the sensor inside the map
    SensorInfo info("DynamicPitot", 1000 / ANALOG_BARO_FREQ,
                    bind(&HILSensors::dynamicPressureCallback, this));
    sensorMap.emplace(make_pair(dynamicPressure, info));
}

void HILSensors::imuInit()
{
    // Register the IMU as the fake sensor, passing as parameters the
    // methods to retrieve real data. The sensor is not synchronized, but
    // the sampling thread is always the same.

    imu = new RotatedIMU(
        bind(&HILSensors::getLSM6DSRXLastSample, this),
        bind(&HILSensors::getCalibratedMagnetometerLastSample, this),
        bind(&HILSensors::getLSM6DSRXLastSample, this));

    // Emplace the sensor inside the map (TODO CHANGE PERIOD INTO NON MAGIC)
    SensorInfo info("RotatedIMU", IMU_PERIOD,
                    bind(&HILSensors::imuCallback, this));
    sensorMap.emplace(make_pair(imu, info));
}

void HILSensors::dynamicPressureCallback()
{
    miosix::PauseKernelLock lock;
    Logger::getInstance().log(getDynamicPressureLastSample());
}
void HILSensors::lps22dfCallback()
{
    miosix::PauseKernelLock lock;
    Logger::getInstance().log(getLPS22DFLastSample());
}
void HILSensors::lps28dfw_1Callback()
{
    miosix::PauseKernelLock lock;
    Logger::getInstance().log(
        static_cast<LPS28DFW_1Data>(getLPS28DFW_1LastSample()));
}
void HILSensors::lps28dfw_2Callback()
{
    miosix::PauseKernelLock lock;
    Logger::getInstance().log(
        static_cast<LPS28DFW_2Data>(getLPS28DFW_2LastSample()));
}
void HILSensors::h3lis331dlCallback()
{
    miosix::PauseKernelLock lock;
    Logger::getInstance().log(getH3LIS331DLLastSample());
}
void HILSensors::lis2mdlCallback()
{
    miosix::PauseKernelLock lock;
    Logger::getInstance().log(getLIS2MDLLastSample());
}
void HILSensors::ubxgpsCallback()
{
    miosix::PauseKernelLock lock;
    Logger::getInstance().log(getGPSLastSample());
}
void HILSensors::lsm6dsrxCallback()
{
    miosix::PauseKernelLock lock;
    Logger::getInstance().log(getLSM6DSRXLastSample());
}
void HILSensors::staticPressureCallback()
{
    miosix::PauseKernelLock lock;
    Logger::getInstance().log(getStaticPressureLastSample());
}
void HILSensors::imuCallback()
{
    miosix::PauseKernelLock lock;
    Logger::getInstance().log(getIMULastSample());
}
}  // namespace Payload