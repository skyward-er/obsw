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

#include <HILConfig.h>
#include <Main/Buses.h>
#include <Main/Configs/SensorsConfig.h>
#include <interfaces-impl/hwmapping.h>

using namespace Boardcore;
using namespace std;
using namespace Main::SensorsConfig;
using namespace HILConfig;

namespace Main
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
    miosix::PauseKernelLock lock;
    if (ubxgps == nullptr)
    {
        return UBXGPSData{};
    }

    GPSData gps = static_cast<Boardcore::GPSData>(ubxgps->getLastSample());
    UBXGPSData lastSample;
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
    miosix::PauseKernelLock lock;

    if (lsm6dsrx_accel == nullptr || lsm6dsrx_gyro == nullptr)
    {
        return LSM6DSRXData{};
    }

    HILAccelerometerData accelData = lsm6dsrx_accel->getLastSample();
    HILGyroscopeData gyroData      = lsm6dsrx_gyro->getLastSample();

    LSM6DSRXData lastSample;

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
ADS131M08Data HILSensors::getADS131M08LastSample()
{
    miosix::PauseKernelLock lock;
    // return ads131m08 != nullptr ? ads131m08->getLastSample() :
    // ADS131M08Data{};
    return ADS131M08Data{};
}

Boardcore::MPXH6400AData HILSensors::getDeploymentPressureLastSample()
{
    // TODO:
    return Boardcore::MPXH6400AData{};
}
Boardcore::HSCMRNN015PAData HILSensors::getStaticPressure1LastSample()
{
    miosix::PauseKernelLock lock;
    Boardcore::HSCMRNN015PAData data;

    data.pressureTimestamp = lps22df->getLastSample().pressureTimestamp;
    data.pressure          = lps22df->getLastSample().pressure;

    return data;
}

Boardcore::HSCMRNN015PAData HILSensors::getStaticPressure2LastSample()
{
    miosix::PauseKernelLock lock;
    Boardcore::HSCMRNN015PAData data;

    data.pressureTimestamp = lps22df->getLastSample().pressureTimestamp;
    data.pressure          = lps22df->getLastSample().pressure;

    return data;
}

RotatedIMUData HILSensors::getIMULastSample()
{
    // TODO:
    return RotatedIMUData{};
}

HILSensors::HILSensors(TaskScheduler* sched) : Main::Sensors(sched) {}

bool HILSensors::start()
{
    printf("HILSensors starting\n");
    // Init all the sensors
    temperatureInit();
    printf("temperatureInit\n");
    lps22dfInit();
    printf("lps22dfInit\n");
    lps28dfw_1Init();
    printf("lps28dfw_1Init\n");
    lps28dfw_2Init();
    printf("lps28dfw_2Init\n");
    h3lis331dlInit();
    printf("h3lis331dlInit\n");
    lis2mdlInit();
    printf("lis2mdlInit\n");
    ubxgpsInit();
    printf("ubxgpsInit\n");
    lsm6dsrxInit();
    printf("lsm6dsrxInit\n");
    ads131m08Init();
    printf("ads131m08Init\n");

    // Create sensor manager with populated map and configured scheduler
    manager = new SensorManager(sensorMap, scheduler);
    return manager->start();
}

void HILSensors::stop() { manager->stop(); }

bool HILSensors::isStarted() { return manager->areAllSensorsInitialized(); }

void HILSensors::calibrate() {}

void HILSensors::temperatureInit()
{
    temperature =
        new HILTemperature(N_DATA_TEMP, &Boardcore::ModuleManager::getInstance()
                                             .get<HIL>()
                                             ->simulator->getSensorData()
                                             ->temperature);

    // Emplace the sensor inside the map
    SensorInfo info("TEMP_HIL", 1000 / TEMP_FREQ);

    sensorMap.emplace(make_pair(temperature, info));
}

void HILSensors::lps22dfInit()
{
    // Create sensor instance with configured parameters
    lps22df =
        new HILBarometer(N_DATA_BARO, &Boardcore::ModuleManager::getInstance()
                                           .get<HIL>()
                                           ->simulator->getSensorData()
                                           ->barometer1);

    // Emplace the sensor inside the map
    SensorInfo info("LPS22DF_HIL", LPS22DF_PERIOD,
                    bind(&HILSensors::lps22dfCallback, this));
    sensorMap.emplace(make_pair(lps22df, info));
}

void HILSensors::lps28dfw_1Init()
{
    // Create sensor instance with configured parameters
    lps28dfw_1 =
        new HILBarometer(N_DATA_BARO, &Boardcore::ModuleManager::getInstance()
                                           .get<HIL>()
                                           ->simulator->getSensorData()
                                           ->barometer2);

    // Emplace the sensor inside the map
    SensorInfo info("LPS28DFW_1_HIL", LPS28DFW_PERIOD,
                    bind(&HILSensors::lps28dfw_1Callback, this));
    sensorMap.emplace(make_pair(lps28dfw_1, info));
}

void HILSensors::lps28dfw_2Init()
{
    // Create sensor instance with configured parameters
    lps28dfw_2 =
        new HILBarometer(N_DATA_BARO, &Boardcore::ModuleManager::getInstance()
                                           .get<HIL>()
                                           ->simulator->getSensorData()
                                           ->barometer3);

    // Emplace the sensor inside the map
    SensorInfo info("LPS28DFW_2_HIL", LPS28DFW_PERIOD,
                    bind(&HILSensors::lps28dfw_2Callback, this));
    sensorMap.emplace(make_pair(lps28dfw_2, info));
}

void HILSensors::h3lis331dlInit()
{
    // Create sensor instance with configured parameters
    h3lis331dl = new HILAccelerometer(N_DATA_ACCEL,
                                      &Boardcore::ModuleManager::getInstance()
                                           .get<HIL>()
                                           ->simulator->getSensorData()
                                           ->accelerometer);

    // Emplace the sensor inside the map
    SensorInfo info("H3LIS331DL_HIL", H3LIS331DL_PERIOD,
                    bind(&HILSensors::h3lis331dlCallback, this));
    sensorMap.emplace(make_pair(h3lis331dl, info));
}

void HILSensors::lis2mdlInit()
{
    // Create sensor instance with configured parameters
    lis2mdl = new HILMagnetometer(N_DATA_MAGN,
                                  &Boardcore::ModuleManager::getInstance()
                                       .get<HIL>()
                                       ->simulator->getSensorData()
                                       ->magnetometer);

    // Emplace the sensor inside the map
    SensorInfo info("LIS2MDL_HIL", LIS2MDL_PERIOD,
                    bind(&HILSensors::lis2mdlCallback, this));
    sensorMap.emplace(make_pair(lis2mdl, info));
}

void HILSensors::ubxgpsInit()
{
    // Create sensor instance with configured parameters
    ubxgps = new HILGps(N_DATA_GPS, &Boardcore::ModuleManager::getInstance()
                                         .get<HIL>()
                                         ->simulator->getSensorData()
                                         ->gps);

    // Emplace the sensor inside the map
    SensorInfo info("UBXGPS_HIL", UBXGPS_PERIOD,
                    bind(&HILSensors::ubxgpsCallback, this));
    sensorMap.emplace(make_pair(ubxgps, info));
}

void HILSensors::lsm6dsrxInit()
{
    // Create sensor instance with configured parameters
    lsm6dsrx_accel = new HILAccelerometer(
        N_DATA_ACCEL, &Boardcore::ModuleManager::getInstance()
                           .get<HIL>()
                           ->simulator->getSensorData()
                           ->accelerometer);
    lsm6dsrx_gyro =
        new HILGyroscope(N_DATA_GYRO, &Boardcore::ModuleManager::getInstance()
                                           .get<HIL>()
                                           ->simulator->getSensorData()
                                           ->gyro);

    // Emplace the sensor inside the map
    SensorInfo info("LSM6DSRX", LSM6DSRX_PERIOD,
                    bind(&HILSensors::lsm6dsrxCallback, this));
    sensorMap.emplace(make_pair(lsm6dsrx_accel, info));
    sensorMap.emplace(make_pair(lsm6dsrx_gyro, info));
}

void HILSensors::ads131m08Init()
{
    // Configure the SPI
    // SPIBusConfig config;
    // config.clockDivider = SPI::ClockDivider::DIV_32;

    // Configure the device
    // ADS131M08::Config sensorConfig;
    // sensorConfig.oversamplingRatio     = ADS131M08_OVERSAMPLING_RATIO;
    // sensorConfig.globalChopModeEnabled = ADS131M08_GLOBAL_CHOP_MODE;

    // Create the sensor instance with configured parameters
    // ads131m08 = new ADS131M08(modules.get<Buses>()->spi4,
    //                           miosix::sensors::ADS131::cs::getPin(), config,
    //                           sensorConfig);

    // // Emplace the sensor inside the map
    // SensorInfo info("ADS131M08", ADS131M08_PERIOD,
    //                 bind(&HILSensors::ads131m08Callback, this));
    // sensorMap.emplace(make_pair(ads131m08, info));
}

void HILSensors::deploymentPressureInit() { return; }

void HILSensors::staticPressure1Init() { return; }

void HILSensors::staticPressure2Init() { return; }

void HILSensors::imuInit()
{
    // Register the IMU as the fake sensor, passing as parameters the methods to
    // retrieve real data. The sensor is not synchronized, but the sampling
    // thread is always the same.
    imu = new RotatedIMU(
        bind(&HILAccelerometer::getLastSample, lsm6dsrx_accel),
        bind(&Sensors::getCalibratedMagnetometerLastSample, this),
        bind(&HILGyroscope::getLastSample, lsm6dsrx_gyro));

    // Invert the Y axis on the magnetometer
    Eigen::Matrix3f m{{1, 0, 0}, {0, -1, 0}, {0, 0, 1}};
    imu->addMagTransformation(m);

    // Emplace the sensor inside the map (TODO CHANGE PERIOD INTO NON MAGIC)
    SensorInfo info("RotatedIMU", IMU_PERIOD,
                    bind(&HILSensors::imuCallback, this));
    sensorMap.emplace(make_pair(imu, info));
}

void HILSensors::lps22dfCallback()
{
    miosix::PauseKernelLock lock;
    // Logger::getInstance().log(getLPS22DFLastSample());
}
void HILSensors::lps28dfw_1Callback()
{
    miosix::PauseKernelLock lock;
    // Logger::getInstance().log(getLPS28DFW_1LastSample());
}
void HILSensors::lps28dfw_2Callback()
{
    miosix::PauseKernelLock lock;
    // Logger::getInstance().log(getLPS28DFW_2LastSample());
}
void HILSensors::h3lis331dlCallback()
{
    miosix::PauseKernelLock lock;
    // Logger::getInstance().log(getH3LIS331DLLastSample());
}
void HILSensors::lis2mdlCallback()
{
    miosix::PauseKernelLock lock;
    // Logger::getInstance().log(getLIS2MDLLastSample());
}
void HILSensors::ubxgpsCallback()
{
    miosix::PauseKernelLock lock;
    // Logger::getInstance().log(getGPSLastSample());
}
void HILSensors::lsm6dsrxCallback()
{
    miosix::PauseKernelLock lock;
    // Logger::getInstance().log(getLSM6DSRXLastSample());
}
void HILSensors::ads131m08Callback()
{
    miosix::PauseKernelLock lock;
    // Logger::getInstance().log(getADS131M08LastSample());
}
void HILSensors::deploymentPressureCallback()
{
    miosix::PauseKernelLock lock;
    // Logger::getInstance().log(getDeploymentPressureLastSample());
}
void HILSensors::staticPressure1Callback()
{
    miosix::PauseKernelLock lock;
    Logger::getInstance().log(getStaticPressure1LastSample());
}
void HILSensors::staticPressure2Callback()
{
    miosix::PauseKernelLock lock;
    Logger::getInstance().log(getStaticPressure2LastSample());
}
void HILSensors::imuCallback()
{
    miosix::PauseKernelLock lock;
    Logger::getInstance().log(getIMULastSample());
}
}  // namespace Main